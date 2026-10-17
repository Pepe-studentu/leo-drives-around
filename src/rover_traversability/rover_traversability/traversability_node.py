#!/usr/bin/env python3
"""ROS wrapper around HeightMap (height_map.py): lidar in, height map + costmap out.

Subscribes
  /points                    lidar scan (sensor_msgs/PointCloud2)
  /satellite/oracle_fix      global fix; only its timestamp is used (map-confidence decay)
Publishes
  /traversability/costmap        nav_msgs/OccupancyGrid for Nav2 (0..100)
  /traversability/slope_costmap  debug layer   (if publish_debug_costmaps)
  /traversability/step_costmap   debug layer   (if publish_debug_costmaps)
  /height_map/cloud              fused height of every known cell (only if subscribed)
  /height_map/raw_cloud          current scan in the map frame   (only if subscribed)

All terrain reasoning is in height_map.py; this file only does ROS I/O and TF.
"""
import numpy as np
import rclpy
import rclpy.time
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from rover_traversability.height_map import HeightMap, HeightMapConfig


def quat_to_R(x, y, z, w):
    """Quaternion (x, y, z, w) -> 3x3 rotation matrix."""
    return np.array([
        [1 - 2 * (y * y + z * z),  2 * (x * y - z * w),      2 * (x * z + y * w)],
        [2 * (x * y + z * w),      1 - 2 * (x * x + z * z),  2 * (y * z - x * w)],
        [2 * (x * z - y * w),      2 * (y * z + x * w),      1 - 2 * (x * x + y * y)],
    ])


CLOUD_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]

# Defaults live in height_map.HeightMapConfig; config/traversability.yaml overrides them.
_CFG_DEFAULTS = HeightMapConfig()


class TraversabilityNode(Node):

    def __init__(self):
        super().__init__('traversability_node')

        # --- ROS-level parameters ---
        self.declare_parameter('input_cloud_topic', '/points')
        self.declare_parameter('oracle_topic', '/satellite/oracle_fix')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('rover_frame', 'base_footprint')
        self.declare_parameter('min_range', 0.35)
        self.declare_parameter('max_range', 15.0)
        self.declare_parameter('update_period', 0.50)
        self.declare_parameter('publish_debug_costmaps', True)

        # --- map parameters: one per HeightMapConfig field, same name ---
        for name, default in vars(_CFG_DEFAULTS).items():
            self.declare_parameter(name, float(default))
        cfg = HeightMapConfig(**{n: self.get_parameter(n).value for n in vars(_CFG_DEFAULTS)})
        self.map = HeightMap(cfg)

        self.input_cloud_topic = self.get_parameter('input_cloud_topic').value
        self.oracle_topic = self.get_parameter('oracle_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.rover_frame = self.get_parameter('rover_frame').value
        self.min_range = self.get_parameter('min_range').value
        self.max_range = self.get_parameter('max_range').value
        self.publish_debug_costmaps = self.get_parameter('publish_debug_costmaps').value

        self.bootstrapped = False
        self.last_oracle_stamp = None
        self.last_stamp = None
        self.prev_scan_time = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Own callback groups: cloud_cb may block in lookup_transform while the
        # TF listener keeps running on another executor thread.
        self.cb_cloud = MutuallyExclusiveCallbackGroup()
        self.cb_timer = MutuallyExclusiveCallbackGroup()

        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.create_subscription(PointCloud2, self.input_cloud_topic, self.cloud_cb,
                                 sensor_qos, callback_group=self.cb_cloud)
        self.create_subscription(Odometry, self.oracle_topic, self.oracle_cb,
                                 QoSProfile(depth=10))

        # Costmaps are latched so late subscribers (Nav2, RViz) get the last one.
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/traversability/costmap', latched)
        self.slope_pub = self.create_publisher(OccupancyGrid, '/traversability/slope_costmap', latched)
        self.step_pub = self.create_publisher(OccupancyGrid, '/traversability/step_costmap', latched)
        self.height_cloud_pub = self.create_publisher(PointCloud2, '/height_map/cloud', QoSProfile(depth=1))
        self.raw_cloud_pub = self.create_publisher(PointCloud2, '/height_map/raw_cloud', QoSProfile(depth=1))

        self.create_timer(self.get_parameter('update_period').value, self.on_timer,
                          callback_group=self.cb_timer)

        self.get_logger().info(
            f"traversability_node up: {self.map.W}x{self.map.H} cells @ {cfg.resolution:.2f} m, "
            f"cloud='{self.input_cloud_topic}', slope free/warn/lethal = "
            f"{cfg.slope_free_deg:.0f}/{cfg.slope_warn_deg:.0f}/{cfg.slope_lethal_deg:.0f} deg")

    # ------------------------------------------------------------------ #
    def oracle_cb(self, msg: Odometry):
        self.last_oracle_stamp = msg.header.stamp

    def cloud_cb(self, msg: PointCloud2):
        """Transform one scan into the map frame and fuse it into the height map."""
        xyz = point_cloud2.read_points_numpy(msg, field_names=['x', 'y', 'z'], skip_nans=True)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]    # skip_nans does not drop +/-inf (no-return rays)
        if xyz.shape[0] == 0:
            return
        ranges = np.linalg.norm(xyz, axis=1)
        in_range = (ranges >= self.min_range) & (ranges <= self.max_range)
        xyz, ranges = xyz[in_range], ranges[in_range]
        if xyz.shape[0] == 0:
            return

        stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, msg.header.frame_id, stamp, Duration(seconds=0.10))
        except TransformException:
            try:   # fall back to the latest available transform
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame, msg.header.frame_id, rclpy.time.Time())
            except TransformException:
                return
        t, q = tf.transform.translation, tf.transform.rotation
        pts_map = xyz @ quat_to_R(q.x, q.y, q.z, q.w).T + np.array([t.x, t.y, t.z])
        self.last_stamp = msg.header.stamp

        if not self.bootstrapped:
            ground_z = self.map.bootstrap(t.x, t.y, t.z)
            self.bootstrapped = True
            self.get_logger().info(
                f"Bootstrapped spawn ground at ({t.x:.2f}, {t.y:.2f}), z={ground_z:.2f} m")

        if self.raw_cloud_pub.get_subscription_count() > 0:
            hdr = Header(stamp=msg.header.stamp, frame_id=self.map_frame)
            self.raw_cloud_pub.publish(
                point_cloud2.create_cloud_xyz32(hdr, pts_map.astype(np.float32)))

        # Time since the last global fix (capped at 3 s); 1 s if none seen yet.
        if self.last_oracle_stamp is not None:
            dt = (stamp - rclpy.time.Time.from_msg(self.last_oracle_stamp)).nanoseconds * 1e-9
            dt_oracle = max(0.0, min(dt, 3.0))
        else:
            dt_oracle = 1.0

        if self.prev_scan_time is not None:
            self.map.age(min(max((stamp - self.prev_scan_time).nanoseconds * 1e-9, 0.0), 1.0))
        self.prev_scan_time = stamp
        self.map.integrate(pts_map, ranges, dt_oracle)

    # ------------------------------------------------------------------ #
    def on_timer(self):
        stamp = self.last_stamp if self.last_stamp is not None else self.get_clock().now().to_msg()

        if self.height_cloud_pub.get_subscription_count() > 0:
            pts = self.map.cloud_points()
            if pts.shape[0] > 0:
                hdr = Header(stamp=stamp, frame_id=self.map_frame)
                self.height_cloud_pub.publish(point_cloud2.create_cloud(hdr, CLOUD_FIELDS, pts))

        rover_xy = None
        try:
            p = self.tf_buffer.lookup_transform(
                self.map_frame, self.rover_frame, rclpy.time.Time()).transform.translation
            rover_xy = (p.x, p.y)
        except TransformException:
            pass

        cost, slope_layer, step_layer = self.map.compute_costmaps(rover_xy)
        self.costmap_pub.publish(self._occ_msg(cost, stamp))
        if self.publish_debug_costmaps:
            self.slope_pub.publish(self._occ_msg(slope_layer, stamp))
            self.step_pub.publish(self._occ_msg(step_layer, stamp))

    def _occ_msg(self, data, stamp):
        cfg = self.map.cfg
        msg = OccupancyGrid()
        msg.header = Header(stamp=stamp, frame_id=self.map_frame)
        msg.info.map_load_time = stamp
        msg.info.resolution = float(cfg.resolution)
        msg.info.width = int(self.map.W)
        msg.info.height = int(self.map.H)
        msg.info.origin.position.x = float(cfg.origin_x)
        msg.info.origin.position.y = float(cfg.origin_y)
        msg.info.origin.orientation.w = 1.0
        msg.data = data.ravel().tolist()
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = TraversabilityNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
