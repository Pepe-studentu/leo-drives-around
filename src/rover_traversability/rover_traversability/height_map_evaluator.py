#!/usr/bin/env python3
"""Toy-World Perfect Height Map Generator and Real-Time Accuracy Evaluator.

Compares live accumulated terrain elevation from /height_map/cloud against
the known analytical ground truth of the Gazebo calibration world:
- Flat ground: z = 0.0 m
- 20 deg ramp: toe at x = 1.5, run 3m to x = 4.5, y in [-4, 4], z(x) = (x - 1.5) * tan(20 deg)
- Step boxes: box015 (z=0.15), box030 (z=0.30), box045 (z=0.45)

Publishes:
- /height_map/ground_truth_cloud (PointCloud2): Perfect synthetic world geometry
- /height_map/diff_cloud (PointCloud2): Measured points with intensity = height error (m)
"""
import argparse
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header

TAN20 = float(np.tan(np.radians(20.0)))


def get_ground_truth_height(x: float, y: float) -> tuple[float, str]:
    """Analytical ground truth elevation z and region label for any (x, y) coordinate."""
    # Ramp: toe at x=1.5, top at x=4.5, y in [-4.0, 4.0]
    if 1.5 <= x <= 4.5 and -4.0 <= y <= 4.0:
        return (x - 1.5) * TAN20, "RAMP"
    # Box 0.15m: center (0.0, -2.0), size 1.0x1.0
    if -0.5 <= x <= 0.5 and -2.5 <= y <= -1.5:
        return 0.15, "BOX015"
    # Box 0.30m: center (0.0, 2.0), size 1.0x1.0
    if -0.5 <= x <= 0.5 and 1.5 <= y <= 2.5:
        return 0.30, "BOX030"
    # Box 0.45m: center (-2.7, 0.0), size 1.0x1.0
    if -3.2 <= x <= -2.2 and -0.5 <= y <= 0.5:
        return 0.45, "BOX045"
    # Walls (room perimeter)
    if x <= -4.4 or x >= 5.9 or y <= -5.9 or y >= 5.9:
        return 2.0, "WALL"
    # Flat ground everywhere else inside room
    return 0.0, "GROUND"


def create_cloud_xyz_intensity(header: Header, points: np.ndarray) -> PointCloud2:
    """Create PointCloud2 from (N, 4) array [x, y, z, intensity]."""
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    data = points.astype(np.float32).tobytes()
    return PointCloud2(
        header=header,
        height=1,
        width=len(points),
        is_dense=True,
        is_bigendian=False,
        fields=fields,
        point_step=16,
        row_step=16 * len(points),
        data=data,
    )


class HeightMapEvaluator(Node):

    def __init__(self, once: bool = False):
        super().__init__('height_map_evaluator')
        self.once = once
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('map_size_x', 50.0)
        self.declare_parameter('map_size_y', 50.0)
        self.declare_parameter('origin_x', -25.0)
        self.declare_parameter('origin_y', -25.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.resolution = self.get_parameter('resolution').value
        self.map_size_x = self.get_parameter('map_size_x').value
        self.map_size_y = self.get_parameter('map_size_y').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value

        # Precompute synthetic ground truth pointcloud for visualization
        self.gt_cloud_msg = self._generate_gt_cloud()

        # Publishers
        self.gt_cloud_pub = self.create_publisher(
            PointCloud2, '/height_map/ground_truth_cloud', 1)
        self.diff_cloud_pub = self.create_publisher(
            PointCloud2, '/height_map/diff_cloud', 1)

        # Subscriber
        self.sub = self.create_subscription(
            PointCloud2, '/height_map/cloud', self.cloud_callback, 1)

        # Periodic timer to publish ground truth cloud
        self.timer = self.create_timer(2.0, self.on_timer)
        self.last_report_time = 0.0

        self.get_logger().info(
            f"HeightMapEvaluator online. Grid: {self.map_size_x}x{self.map_size_y}m @ {self.resolution}m. "
            "Listening on /height_map/cloud..."
        )

    def _generate_gt_cloud(self) -> PointCloud2:
        """Generate dense ground-truth surface across the active toy world."""
        xs = np.arange(-5.0, 6.0, 0.10)
        ys = np.arange(-5.0, 5.0, 0.10)
        grid_x, grid_y = np.meshgrid(xs, ys)
        pts = []
        for x, y in zip(grid_x.ravel(), grid_y.ravel()):
            z, _ = get_ground_truth_height(float(x), float(y))
            pts.append([x, y, z, z])
        pts_arr = np.array(pts, dtype=np.float32)
        hdr = Header(stamp=self.get_clock().now().to_msg(), frame_id=self.map_frame)
        return create_cloud_xyz_intensity(hdr, pts_arr)

    def on_timer(self):
        self.gt_cloud_msg.header.stamp = self.get_clock().now().to_msg()
        self.gt_cloud_pub.publish(self.gt_cloud_msg)

    def cloud_callback(self, msg: PointCloud2):
        pts = point_cloud2.read_points_numpy(
            msg, field_names=['x', 'y', 'z'], skip_nans=True)
        if len(pts) == 0:
            return

        diff_pts = []
        errors = []
        errors_by_region = {"GROUND": [], "RAMP": [], "BOXES": []}

        for p in pts:
            x, y, z_meas = float(p[0]), float(p[1]), float(p[2])
            z_true, region = get_ground_truth_height(x, y)
            err = z_meas - z_true
            diff_pts.append([x, y, z_meas, err])
            errors.append(abs(err))

            if region == "GROUND":
                errors_by_region["GROUND"].append(abs(err))
            elif region == "RAMP":
                errors_by_region["RAMP"].append(abs(err))
            else:
                errors_by_region["BOXES"].append(abs(err))

        # Publish error diff cloud (intensity = signed error in meters)
        hdr = Header(stamp=msg.header.stamp, frame_id=self.map_frame)
        diff_arr = np.array(diff_pts, dtype=np.float32)
        self.diff_cloud_pub.publish(create_cloud_xyz_intensity(hdr, diff_arr))

        # Throttled terminal reporting (every 2.5 seconds)
        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self.last_report_time >= 2.5 or self.once:
            self.last_report_time = now_sec
            self.print_report(errors, errors_by_region)
            if self.once:
                sys.exit(0)

    def print_report(self, errors: list, regions: dict):
        total_n = len(errors)
        mae = float(np.mean(errors))
        rmse = float(np.sqrt(np.mean(np.square(errors))))
        max_err = float(np.max(errors))

        print("\n" + "=" * 65)
        print(f"  PERFECT GROUND-TRUTH HEIGHT MAP EVALUATION ({total_n} cells)")
        print("=" * 65)
        print(f"  Overall MAE : {mae * 1000.0:6.1f} mm  ({mae:.4f} m)")
        print(f"  Overall RMSE: {rmse * 1000.0:6.1f} mm  ({rmse:.4f} m)")
        print(f"  Max Error   : {max_err * 1000.0:6.1f} mm  ({max_err:.4f} m)")
        print("-" * 65)
        for name, errs in regions.items():
            if errs:
                r_mae = float(np.mean(errs)) * 1000.0
                r_max = float(np.max(errs)) * 1000.0
                print(f"  {name:<8}: count={len(errs):>5} | MAE={r_mae:5.1f} mm | MaxErr={r_max:5.1f} mm")
            else:
                print(f"  {name:<8}: (no data painted yet)")
        print("=" * 65 + "\n")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--once', action='store_true', help='Print one report and exit')
    args, unknown = parser.parse_known_args()

    rclpy.init()
    node = HeightMapEvaluator(once=args.once)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
