#!/usr/bin/env python3
"""Confidence-weighted 3D height-map visualization and accumulator.

Bins raw LiDAR scans into a 2.5D elevation grid in the global map frame
and publishes the accumulated terrain as a PointCloud2 (/height_map/cloud).

Update Logic:
- Unvisited cells (-inf): directly overridden with new observations.
- Visited cells: updated weighted by real-time EKF confidence alpha in [min_conf, 1.0]:
    height = (1 - alpha) * height_old + alpha * height_obs
- High Confidence (alpha = 1.0): when stationary or rolling with solid traction,
  new observations directly update the map (no sluggish running average lag).
- Low Confidence (alpha -> 0.05): when wheels skid (wheel encoder speed vs EKF
  robot speed discrepancy) or EKF covariance spikes, updates only nudge cells
  slightly so skidding cannot distort or inflate obstacles/ramps.
"""
import numpy as np
import rclpy
import rclpy.time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


def quat_to_R(x, y, z, w):
    """Quaternion (x, y, z, w) -> 3x3 rotation matrix."""
    return np.array([
        [1 - 2 * (y * y + z * z),  2 * (x * y - z * w),      2 * (x * z + y * w)],
        [2 * (x * y + z * w),      1 - 2 * (x * x + z * z),  2 * (y * z - x * w)],
        [2 * (x * z - y * w),      2 * (y * z + x * w),      1 - 2 * (x * x + y * y)],
    ])


class HeightMapVizNode(Node):

    def __init__(self):
        super().__init__('height_map_viz_node')

        self.declare_parameter('input_cloud_topic', '/points')
        self.declare_parameter('wheel_odom_topic', '/odom')
        self.declare_parameter('pose_topic', '/odometry/global')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('map_size_x', 60.0)
        self.declare_parameter('map_size_y', 60.0)
        self.declare_parameter('origin_x', -30.0)
        self.declare_parameter('origin_y', -30.0)
        self.declare_parameter('max_range', 20.0)
        self.declare_parameter('min_range', 0.35)           # ignore rover chassis
        self.declare_parameter('update_period', 0.20)
        self.declare_parameter('min_confidence', 0.05)
        self.declare_parameter('stationary_vel_threshold', 0.03)  # m/s
        self.declare_parameter('slip_threshold', 0.04)            # m/s
        self.declare_parameter('slip_scale', 0.10)                # m/s
        self.declare_parameter('cov_threshold', 0.02)             # m^2
        self.declare_parameter('cov_scale', 0.05)                 # m^2

        def _str(name):
            return self.get_parameter(name).get_parameter_value().string_value

        def _float(name):
            return self.get_parameter(name).get_parameter_value().double_value

        self.input_cloud_topic = _str('input_cloud_topic')
        self.wheel_odom_topic = _str('wheel_odom_topic')
        self.pose_topic = _str('pose_topic')
        self.map_frame = _str('map_frame')
        self.resolution = _float('resolution')
        self.map_size_x = _float('map_size_x')
        self.map_size_y = _float('map_size_y')
        self.origin_x = _float('origin_x')
        self.origin_y = _float('origin_y')
        self.max_range = _float('max_range')
        self.min_range = _float('min_range')
        self.update_period = _float('update_period')
        self.min_confidence = _float('min_confidence')
        self.stationary_vel_threshold = _float('stationary_vel_threshold')
        self.slip_threshold = _float('slip_threshold')
        self.slip_scale = _float('slip_scale')
        self.cov_threshold = _float('cov_threshold')
        self.cov_scale = _float('cov_scale')

        self.W = int(round(self.map_size_x / self.resolution))
        self.H = int(round(self.map_size_y / self.resolution))
        self.height = np.full((self.H, self.W), -np.inf, dtype=np.float32)
        self.hits = np.zeros((self.H, self.W), dtype=np.int32)

        # Odometry and EKF state tracking for skid detection
        self.wheel_vx = 0.0
        self.wheel_wz = 0.0
        self.robot_vx = 0.0
        self.robot_wz = 0.0
        self.pos_var = 0.0
        self.yaw_var = 0.0
        self.last_stamp = None
        self.last_confidence = 1.0
        self.last_status = "STATIONARY"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cb_cloud = MutuallyExclusiveCallbackGroup()
        self.cb_timer = MutuallyExclusiveCallbackGroup()

        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.input_cloud_topic, self.cloud_cb, sensor_qos,
            callback_group=self.cb_cloud)

        self.wheel_odom_sub = self.create_subscription(
            Odometry, self.wheel_odom_topic, self.wheel_odom_cb, QoSProfile(depth=5))

        self.pose_sub = self.create_subscription(
            Odometry, self.pose_topic, self.pose_cb, QoSProfile(depth=5))

        self.raw_cloud_pub = self.create_publisher(
            PointCloud2, '/height_map/raw_cloud', QoSProfile(depth=1))
        self.height_cloud_pub = self.create_publisher(
            PointCloud2, '/height_map/cloud', QoSProfile(depth=1))

        self.timer = self.create_timer(
            self.update_period, self.on_timer, callback_group=self.cb_timer)

        self.get_logger().info(
            f"height_map_viz_node up: cloud='{self.input_cloud_topic}' "
            f"-> '/height_map/cloud', frame='{self.map_frame}', "
            f"grid={self.W}x{self.H} @ {self.resolution} m/cell (confidence-weighted fusion)"
        )

    # ------------------------------------------------------------------ #
    def wheel_odom_cb(self, msg: Odometry):
        """Track instantaneous wheel speed from wheel encoders."""
        self.wheel_vx = float(msg.twist.twist.linear.x)
        self.wheel_wz = float(msg.twist.twist.angular.z)

    # ------------------------------------------------------------------ #
    def pose_cb(self, msg: Odometry):
        """Track filtered robot speed and EKF covariance from /odometry/global."""
        self.robot_vx = float(msg.twist.twist.linear.x)
        self.robot_wz = float(msg.twist.twist.angular.z)
        cov = msg.pose.covariance
        self.pos_var = float(cov[0] + cov[7])
        self.yaw_var = float(cov[35])

    # ------------------------------------------------------------------ #
    def compute_confidence(self):
        """Compute update confidence alpha in [min_confidence, 1.0].

        - 1.0: Robot stationary or moving with solid traction (direct update).
        - ~0.05: Wheels skidding or EKF covariance inflated (nudge only).
        """
        # 1. Stationary check: zero wheel speed and zero estimated speed
        is_wheel_stopped = (abs(self.wheel_vx) < self.stationary_vel_threshold and
                            abs(self.wheel_wz) < self.stationary_vel_threshold)
        is_robot_stopped = (abs(self.robot_vx) < self.stationary_vel_threshold and
                            abs(self.robot_wz) < self.stationary_vel_threshold)
        if is_wheel_stopped and is_robot_stopped:
            return 1.0, "STATIONARY", 0.0

        # 2. Wheel slip discrepancy: wheel speed vs EKF estimated speed
        linear_slip = max(0.0, abs(self.wheel_vx - self.robot_vx) - self.slip_threshold)
        angular_slip = max(0.0, abs(self.wheel_wz - self.robot_wz) - self.slip_threshold)
        slip_penalty = (linear_slip / self.slip_scale) + (angular_slip / (self.slip_scale * 2.0))
        slip_factor = np.exp(-slip_penalty)

        # 3. EKF covariance penalty
        cov_excess = max(0.0, self.pos_var - self.cov_threshold)
        cov_factor = np.exp(-cov_excess / self.cov_scale)

        confidence = float(np.clip(slip_factor * cov_factor, self.min_confidence, 1.0))

        if confidence < 0.25:
            status = "SKID_DETECTED"
        elif confidence < 0.80:
            status = "SLIP_WARNING"
        else:
            status = "TRACTION_OK"

        return confidence, status, linear_slip

    # ------------------------------------------------------------------ #
    def cloud_cb(self, msg: PointCloud2):
        """Read scan, transform to map, and update height grid with confidence weighting."""
        xyz = point_cloud2.read_points_numpy(
            msg, field_names=["x", "y", "z"], skip_nans=True)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if xyz.shape[0] == 0:
            return

        ranges = np.linalg.norm(xyz, axis=1)
        valid_range = (ranges >= self.min_range) & (ranges <= self.max_range)
        xyz = xyz[valid_range]
        if xyz.shape[0] == 0:
            return

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                rclpy.time.Time.from_msg(msg.header.stamp),
                Duration(seconds=0.10),
            )
        except TransformException:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.map_frame,
                    msg.header.frame_id,
                    rclpy.time.Time(),
                )
            except TransformException as e:
                self.get_logger().warn(f"no TF yet: {e}", throttle_duration_sec=2.0)
                return

        q = tf.transform.rotation
        t = tf.transform.translation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        pts_map = xyz @ R.T + np.array([t.x, t.y, t.z])
        self.last_stamp = msg.header.stamp

        # Republish raw cloud in map frame for visual verification in RViz
        hdr = Header(stamp=msg.header.stamp, frame_id=self.map_frame)
        self.raw_cloud_pub.publish(
            point_cloud2.create_cloud_xyz32(hdr, pts_map.astype(np.float32)))

        # Project points into height grid
        xs, ys, zs = pts_map[:, 0], pts_map[:, 1], pts_map[:, 2]
        cols = ((xs - self.origin_x) / self.resolution).astype(np.int32)
        rows = ((ys - self.origin_y) / self.resolution).astype(np.int32)
        m = (cols >= 0) & (cols < self.W) & (rows >= 0) & (rows < self.H)
        rows, cols, zs = rows[m], cols[m], zs[m].astype(np.float32)
        if rows.size == 0:
            return

        # Instantaneous elevation observation for this scan
        obs = np.full((self.H, self.W), -np.inf, dtype=np.float32)
        np.maximum.at(obs, (rows, cols), zs)
        seen = np.isfinite(obs)

        # Compute confidence from wheel slip and EKF state
        confidence, status, slip = self.compute_confidence()
        self.last_confidence = confidence
        self.last_status = status

        self.get_logger().info(
            f"[{status}] conf={confidence:.2f} | wheel_v={self.wheel_vx:.2f} "
            f"robot_v={self.robot_vx:.2f} slip={slip:.2f} pos_var={self.pos_var:.4f}",
            throttle_duration_sec=2.0
        )

        # Unvisited cells (-inf): directly override with observation
        new_cells = seen & (self.height == -np.inf)
        self.height[new_cells] = obs[new_cells]

        # Visited cells: update weighted by confidence
        # When confident (alpha = 1.0): directly sets to obs (no sluggish EMA)
        # When skidding (alpha = 0.05): nudges cells only slightly (5%)
        old_cells = seen & (self.height != -np.inf)
        self.height[old_cells] = (1.0 - confidence) * self.height[old_cells] + confidence * obs[old_cells]

        self.hits[seen] += 1

    # ------------------------------------------------------------------ #
    def on_timer(self):
        """Publish accumulated height map as a 3D PointCloud2."""
        filled = np.isfinite(self.height) & (self.hits > 0)
        rr, cc = np.nonzero(filled)
        if rr.size == 0:
            return
        cx = self.origin_x + (cc.astype(np.float32) + 0.5) * self.resolution
        cy = self.origin_y + (rr.astype(np.float32) + 0.5) * self.resolution
        cz = self.height[rr, cc]

        stamp = self.last_stamp if self.last_stamp is not None else self.get_clock().now().to_msg()
        hdr = Header(stamp=stamp, frame_id=self.map_frame)
        fields = [
            point_cloud2.PointField(name='x', offset=0, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='y', offset=4, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='z', offset=8, datatype=point_cloud2.PointField.FLOAT32, count=1),
            point_cloud2.PointField(name='intensity', offset=12, datatype=point_cloud2.PointField.FLOAT32, count=1),
        ]
        cloud_pts = np.stack([cx, cy, cz, cz], axis=1).astype(np.float32)
        self.height_cloud_pub.publish(
            point_cloud2.create_cloud(hdr, fields, cloud_pts))


def main(args=None):
    rclpy.init(args=args)
    node = HeightMapVizNode()
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
