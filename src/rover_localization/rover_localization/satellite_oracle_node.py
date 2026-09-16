#!/usr/bin/env python3
"""Planetary Satellite Oracle Node.

Simulates a low-rate (0.5 Hz) orbital DEM / satellite beacon that provides
periodic absolute 3D position fixes (X, Y, Z, Yaw) on Mars.
"""

import math
import rclpy
from rclpy.node import Node
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


def euler_from_quaternion(q: Quaternion):
    sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = math.asin(max(-1.0, min(1.0, sinp)))

    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def rotate_vector_by_quaternion(vx, vy, vz, q):
    # v' = q * v * q^-1
    # Standard quaternion vector rotation
    qx, qy, qz, qw = q.x, q.y, q.z, q.w
    ix =  qw * vx + qy * vz - qz * vy
    iy =  qw * vy + qz * vx - qx * vz
    iz =  qw * vz + qx * vy - qy * vx
    iw = -qx * vx - qy * vy - qz * vz

    rx = ix * qw + iw * -qx + iy * -qz - iz * -qy
    ry = iy * qw + iw * -qy + iz * -qx - ix * -qz
    rz = iz * qw + iw * -qz + ix * -qy - iy * -qx
    return rx, ry, rz


class SatelliteOracleNode(Node):

    def __init__(self):
        super().__init__('satellite_oracle_node')

        self.declare_parameter('update_rate', 0.5)      # 0.5 Hz (every 2.0s)
        self.declare_parameter('target_model', 'leo_rover')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('oracle_topic', '/satellite/oracle_fix')
        self.declare_parameter('base_link_offset_z', 0.0)  # Gazebo model root is already base_footprint

        self.update_rate = float(self.get_parameter('update_rate').value)
        self.target_model = self.get_parameter('target_model').value
        self.map_frame = self.get_parameter('map_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        oracle_topic = self.get_parameter('oracle_topic').value
        self.base_offset_z = float(self.get_parameter('base_link_offset_z').value)

        self.oracle_pub = self.create_publisher(Odometry, oracle_topic, 10)

        self.create_subscription(
            TFMessage,
            '/gazebo/dynamic_pose',
            self.dynamic_pose_cb,
            10
        )

        self.latest_transform = None
        timer_period = 1.0 / self.update_rate
        self.timer = self.create_timer(timer_period, self.on_timer)

        self.get_logger().info(
            f"Satellite Oracle initialized at {self.update_rate} Hz. "
            f"Publishing absolute 3D fixes on '{oracle_topic}'"
        )

    def dynamic_pose_cb(self, msg: TFMessage):
        if not msg.transforms:
            return

        target_tf = None
        for tf in msg.transforms:
            if tf.child_frame_id == self.target_model:
                target_tf = tf
                break

        if target_tf is None:
            if len(msg.transforms) > 1 and (
                msg.transforms[0].transform.translation.x == 0.0 and
                msg.transforms[0].transform.translation.y == 0.0 and
                msg.transforms[0].transform.translation.z == 0.0
            ):
                target_tf = msg.transforms[1]
            else:
                target_tf = msg.transforms[0]

        self.latest_transform = target_tf

    def on_timer(self):
        if self.latest_transform is None:
            return

        tf = self.latest_transform
        q = tf.transform.rotation
        bx = tf.transform.translation.x
        by = tf.transform.translation.y
        bz = tf.transform.translation.z

        # Rotate body-frame offset to world-frame
        dx, dy, dz = rotate_vector_by_quaternion(0.0, 0.0, self.base_offset_z, q)
        footprint_x = bx + dx
        footprint_y = by + dy
        footprint_z = bz + dz

        now = self.get_clock().now().to_msg()
        odom_msg = Odometry()
        odom_msg.header.stamp = now
        odom_msg.header.frame_id = self.map_frame
        odom_msg.child_frame_id = self.base_frame

        odom_msg.pose.pose.position.x = footprint_x
        odom_msg.pose.pose.position.y = footprint_y
        odom_msg.pose.pose.position.z = footprint_z
        odom_msg.pose.pose.orientation = q

        # High-confidence oracle covariance
        odom_msg.pose.covariance[0] = 0.01
        odom_msg.pose.covariance[7] = 0.01
        odom_msg.pose.covariance[14] = 0.01
        odom_msg.pose.covariance[35] = 0.01

        self.oracle_pub.publish(odom_msg)

        r, p, yaw = euler_from_quaternion(q)
        self.get_logger().info(
            f"[Satellite Oracle 0.5Hz] Absolute 3D Fix: "
            f"X={footprint_x:+.2f}, Y={footprint_y:+.2f}, Z={footprint_z:+.2f} m | "
            f"Yaw={math.degrees(yaw):+.1f}deg"
        )


def main(args=None):
    rclpy.init(args=args)
    node = SatelliteOracleNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
