#!/usr/bin/env python3
"""Localization accuracy benchmark and verification tool.

Compares ground truth pose from Gazebo against the state estimated by the
localization stack (/odometry/global and TF map -> base_footprint).

Can be run in:
1. Live monitoring mode:
     ros2 run rover_localization benchmark_loc
2. Automated test suite mode:
     ros2 run rover_localization benchmark_loc --ros-args -p auto:=true
"""
import argparse
import json
import math
import sys
import time
import numpy as np

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener


def quat_to_yaw(qx, qy, qz, qw):
    """Compute planar yaw angle in radians from a quaternion."""
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    return math.atan2(siny_cosp, cosy_cosp)


def normalize_angle_deg(ang_deg):
    """Normalize angle in degrees to [-180, 180]."""
    return (ang_deg + 180.0) % 360.0 - 180.0


class LocalizationBenchmark(Node):
    def __init__(self):
        super().__init__('benchmark_loc')

        self.declare_parameter('auto', False)
        self.declare_parameter('duration', 20.0)
        self.declare_parameter('output_report', '/home/ws/localization_benchmark_report.json')

        self.is_auto = self.get_parameter('auto').get_parameter_value().bool_value
        self.total_duration = self.get_parameter('duration').get_parameter_value().double_value
        self.report_path = self.get_parameter('output_report').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Ground truth pose storage
        self.gt_pose = None
        self.gt_time = None
        self.create_subscription(PoseStamped, '/ground_truth/pose', self.gt_cb, 10)

        # Global odometry storage
        self.est_odom = None
        self.create_subscription(Odometry, '/odometry/global', self.est_odom_cb, 10)

        # Local odometry storage
        self.loc_odom = None
        self.create_subscription(Odometry, '/odometry/local', self.loc_odom_cb, 10)

        # Metrics accumulators
        self.pos_errors = []
        self.yaw_errors = []
        self.timestamps = []
        self.samples = []

        self.t0 = time.time()
        self.timer = self.create_timer(0.1, self.tick)

        self.get_logger().info("Localization Benchmark Node started.")
        if self.is_auto:
            self.get_logger().info("Mode: Automated trajectory benchmark.")
        else:
            self.get_logger().info("Mode: Live real-time monitoring.")

    def gt_cb(self, msg: PoseStamped):
        p = msg.pose.position
        o = msg.pose.orientation
        yaw = quat_to_yaw(o.x, o.y, o.z, o.w)
        self.gt_pose = (p.x, p.y, p.z, yaw)
        self.gt_time = msg.header.stamp

    def est_odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = quat_to_yaw(o.x, o.y, o.z, o.w)
        self.est_odom = (p.x, p.y, p.z, yaw)

    def loc_odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        o = msg.pose.pose.orientation
        yaw = quat_to_yaw(o.x, o.y, o.z, o.w)
        self.loc_odom = (p.x, p.y, p.z, yaw)

    def get_tf_estimated_pose(self):
        try:
            tr = self.tf_buffer.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            t = tr.transform.translation
            r = tr.transform.rotation
            yaw = quat_to_yaw(r.x, r.y, r.z, r.w)
            return t.x, t.y, t.z, yaw
        except Exception:
            return self.est_odom

    def tick(self):
        t = time.time() - self.t0

        # In auto mode, drive pre-programmed motion phases
        current_phase = "MONITOR"
        if self.is_auto:
            tw = Twist()
            if t < 3.0:
                current_phase = "IDLE (Warmup)"
            elif t < 8.0:
                current_phase = "DRIVE (Forward)"
                tw.linear.x = 0.35
            elif t < 12.0:
                current_phase = "ROTATE (Turn Left)"
                tw.angular.z = 0.5
            elif t < 16.0:
                current_phase = "DRIVE (Curved)"
                tw.linear.x = 0.25
                tw.angular.z = -0.3
            elif t < 19.0:
                current_phase = "REVERSE"
                tw.linear.x = -0.25
            else:
                current_phase = "SETTLE (Stop)"
                tw.linear.x = 0.0
                tw.angular.z = 0.0
            self.cmd_pub.publish(tw)

            if t >= self.total_duration:
                self.cmd_pub.publish(Twist())
                self.finish_benchmark()
                return

        # Compare ground truth with estimated
        est = self.get_tf_estimated_pose()
        if self.gt_pose is None or est is None:
            return

        tx, ty, tz, tyaw = self.gt_pose
        ex, ey, ez, eyaw = est

        dx = ex - tx
        dy = ey - ty
        dz = ez - tz
        pos_err_2d = math.hypot(dx, dy)
        pos_err_3d = math.sqrt(dx * dx + dy * dy + dz * dz)

        yaw_err_deg = abs(normalize_angle_deg(math.degrees(eyaw - tyaw)))

        self.pos_errors.append(pos_err_2d)
        self.yaw_errors.append(yaw_err_deg)
        self.timestamps.append(t)
        self.samples.append({
            'time': round(t, 2),
            'phase': current_phase,
            'true_x': round(tx, 3), 'true_y': round(ty, 3), 'true_yaw_deg': round(math.degrees(tyaw), 1),
            'est_x': round(ex, 3), 'est_y': round(ey, 3), 'est_yaw_deg': round(math.degrees(eyaw), 1),
            'pos_err_2d_m': round(pos_err_2d, 3),
            'yaw_err_deg': round(yaw_err_deg, 2)
        })

        # Print line at ~5 Hz
        if len(self.samples) % 2 == 0:
            mean_p = np.mean(self.pos_errors)
            print(f"t={t:5.1f}s [{current_phase:<16s}] | "
                  f"True=({tx:6.2f}, {ty:6.2f}, {math.degrees(tyaw):5.1f}°) | "
                  f"Est=({ex:6.2f}, {ey:6.2f}, {math.degrees(eyaw):5.1f}°) | "
                  f"Δpos={pos_err_2d:5.3f}m  Δyaw={yaw_err_deg:4.1f}° | "
                  f"MeanErr={mean_p:5.3f}m", flush=True)

    def finish_benchmark(self):
        self.cmd_pub.publish(Twist())
        print("\n" + "=" * 75)
        print("🏆 LOCALIZATION ACCURACY BENCHMARK REPORT (GAZEBO GROUND TRUTH COMPARISON)")
        print("=" * 75)

        if not self.pos_errors:
            print("❌ Error: No samples recorded during benchmark.")
            rclpy.shutdown()
            return

        pos_arr = np.array(self.pos_errors)
        yaw_arr = np.array(self.yaw_errors)

        mean_pos = float(np.mean(pos_arr))
        max_pos = float(np.max(pos_arr))
        rmse_pos = float(np.sqrt(np.mean(pos_arr ** 2)))

        mean_yaw = float(np.mean(yaw_arr))
        max_yaw = float(np.max(yaw_arr))
        rmse_yaw = float(np.sqrt(np.mean(yaw_arr ** 2)))

        p95_pos = float(np.percentile(pos_arr, 95))
        p95_yaw = float(np.percentile(yaw_arr, 95))

        # Check navigation tolerance criteria:
        # For outdoor waypoint navigation, position RMSE < 0.25m and Yaw RMSE < 8.0° is excellent.
        pos_pass = rmse_pos <= 0.25
        yaw_pass = rmse_yaw <= 8.0
        overall_pass = pos_pass and yaw_pass

        status_icon = "✅ PASS" if overall_pass else "❌ FAIL"
        print(f"  • Total Evaluation Duration : {self.timestamps[-1]:.1f} seconds")
        print(f"  • Total Synchronized Samples: {len(self.pos_errors)}")
        print(f"  • Position Error (Mean)     : {mean_pos:.4f} m")
        print(f"  • Position Error (RMSE)     : {rmse_pos:.4f} m (Target: <= 0.25m)")
        print(f"  • Position Error (95th %)   : {p95_pos:.4f} m")
        print(f"  • Position Error (Max)      : {max_pos:.4f} m")
        print(f"  • Yaw Error (Mean)          : {mean_yaw:.2f}°")
        print(f"  • Yaw Error (RMSE)          : {rmse_yaw:.2f}° (Target: <= 8.0°)")
        print(f"  • Yaw Error (Max)           : {max_yaw:.2f}°")
        print(f"  • Benchmark Status          : {status_icon}")
        print("=" * 75)

        report = {
            'status': 'PASS' if overall_pass else 'FAIL',
            'duration_sec': round(self.timestamps[-1], 2),
            'samples_count': len(self.pos_errors),
            'position_metrics': {
                'mean_m': round(mean_pos, 4),
                'rmse_m': round(rmse_pos, 4),
                'p95_m': round(p95_pos, 4),
                'max_m': round(max_pos, 4),
                'passed': pos_pass
            },
            'yaw_metrics': {
                'mean_deg': round(mean_yaw, 3),
                'rmse_deg': round(rmse_yaw, 3),
                'p95_deg': round(p95_yaw, 3),
                'max_deg': round(max_yaw, 3),
                'passed': yaw_pass
            },
            'samples': self.samples
        }

        try:
            with open(self.report_path, 'w') as f:
                json.dump(report, f, indent=2)
            print(f"💾 Benchmark report saved to: {self.report_path}\n")
        except Exception as e:
            print(f"⚠️ Could not write report: {e}")

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationBenchmark()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.cmd_pub.publish(Twist())
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
