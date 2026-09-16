#!/usr/bin/env python3
"""Global 3D Corrector Node.

Fuses periodic low-rate 3D absolute fixes (from orbital DEM / satellite oracle)
with high-rate continuous local 3D odometry (KISS-ICP / EKF), broadcasting a smooth,
strictly-upright 3D map -> odom transform according to REP-105.

Pivots corrections directly around the robot's current pose:
- Zero origin lever-arm swing (no lateral jumps when yaw changes).
- Zero tethering (local motion between fixes is 100% uninhibited).
- Rate-limited smooth drift absorption at 20 Hz.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster, Buffer, TransformListener


def wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


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


def quaternion_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class Global3DCorrector(Node):

    def __init__(self):
        super().__init__('global_3d_corrector')

        # Parameters
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('oracle_topic', '/satellite/oracle_fix')
        self.declare_parameter('odom_topic', '/odometry/local')
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('initial_z', 1.594)
        self.declare_parameter('max_trans_vel', 1.0)   # Drift absorption rate: 1.0 m/s
        self.declare_parameter('max_yaw_vel', 1.5)     # Drift absorption yaw rate: 1.5 rad/s (~86 deg/s)

        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        oracle_topic = self.get_parameter('oracle_topic').value
        odom_topic = self.get_parameter('odom_topic').value
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.initial_z = float(self.get_parameter('initial_z').value)
        self.max_trans_vel = float(self.get_parameter('max_trans_vel').value)
        self.max_yaw_vel = float(self.get_parameter('max_yaw_vel').value)

        # Current transform offsets
        self.offset_x = 0.0
        self.offset_y = 0.0
        self.offset_z = self.initial_z
        self.offset_yaw = 0.0

        # Remaining drift error to absorb at robot center (LEVER ARM = 0)
        self.drift_err_x = 0.0
        self.drift_err_y = 0.0
        self.drift_err_z = 0.0
        self.drift_err_yaw = 0.0
        self.initialized = False

        # TF components
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Subscribers
        self.create_subscription(Odometry, oracle_topic, self.oracle_cb, 10)
        self.create_subscription(Odometry, odom_topic, self.odom_cb, 10)

        # Fused global odometry publisher
        self.global_odom_pub = self.create_publisher(Odometry, '/odometry/global', 10)
        self.last_odom_msg = None

        period = 1.0 / self.publish_rate
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info(
            f"Global3DCorrector active (Robot-Centered Pivot): Oracle='{oracle_topic}', Odom='{odom_topic}', "
            f"Rate={self.publish_rate}Hz, max_trans_vel={self.max_trans_vel} m/s, "
            f"max_yaw_vel={math.degrees(self.max_yaw_vel):.1f} deg/s"
        )

    def odom_cb(self, msg: Odometry):
        self.last_odom_msg = msg

    def oracle_cb(self, msg: Odometry):
        gx = msg.pose.pose.position.x
        gy = msg.pose.pose.position.y
        gz = msg.pose.pose.position.z
        _, _, gyaw = euler_from_quaternion(msg.pose.pose.orientation)

        # Look up current odom -> base_footprint transform
        try:
            tf = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.0)
            )
        except Exception:
            return

        ox = tf.transform.translation.x
        oy = tf.transform.translation.y
        oz = tf.transform.translation.z
        _, _, oyaw = euler_from_quaternion(tf.transform.rotation)

        if not self.initialized:
            # Immediate initial lock to world coordinates at robot pose
            init_yaw_offset = wrap_to_pi(gyaw - oyaw)
            cos_y = math.cos(init_yaw_offset)
            sin_y = math.sin(init_yaw_offset)
            rot_ox = ox * cos_y - oy * sin_y
            rot_oy = ox * sin_y + oy * cos_y

            self.offset_yaw = init_yaw_offset
            self.offset_x = gx - rot_ox
            self.offset_y = gy - rot_oy
            self.offset_z = gz - oz

            self.drift_err_x = 0.0
            self.drift_err_y = 0.0
            self.drift_err_z = 0.0
            self.drift_err_yaw = 0.0
            self.initialized = True
            self.get_logger().info(
                f"[Global3DCorrector] Initial lock: "
                f"offset=({self.offset_x:+.3f}, {self.offset_y:+.3f}, {self.offset_z:+.3f}) m | "
                f"offset_yaw={math.degrees(self.offset_yaw):+.2f} deg"
            )
        else:
            # Compute current estimated robot pose in map frame
            cos_curr = math.cos(self.offset_yaw)
            sin_curr = math.sin(self.offset_yaw)
            curr_rx = self.offset_x + (ox * cos_curr - oy * sin_curr)
            curr_ry = self.offset_y + (ox * sin_curr + oy * cos_curr)
            curr_rz = self.offset_z + oz
            curr_ryaw = wrap_to_pi(oyaw + self.offset_yaw)

            # Drift error measured AT THE ROBOT CENTER (lever arm = 0)
            self.drift_err_x = gx - curr_rx
            self.drift_err_y = gy - curr_ry
            self.drift_err_z = gz - curr_rz
            self.drift_err_yaw = wrap_to_pi(gyaw - curr_ryaw)

    def on_timer(self):
        dt = 1.0 / self.publish_rate
        now = self.get_clock().now().to_msg()

        if self.initialized:
            # Compute step towards absorbing remaining drift error
            dist = math.hypot(self.drift_err_x, self.drift_err_y, self.drift_err_z)
            max_step = self.max_trans_vel * dt
            if dist > 1e-4:
                step = min(dist, max_step)
                step_x = (self.drift_err_x / dist) * step
                step_y = (self.drift_err_y / dist) * step
                step_z = (self.drift_err_z / dist) * step
            else:
                step_x = self.drift_err_x
                step_y = self.drift_err_y
                step_z = self.drift_err_z

            max_yaw_step = self.max_yaw_vel * dt
            if abs(self.drift_err_yaw) > 1e-4:
                step_yaw = math.copysign(min(abs(self.drift_err_yaw), max_yaw_step), self.drift_err_yaw)
            else:
                step_yaw = self.drift_err_yaw

            # If there is drift to absorb, apply it around robot center
            has_drift = (abs(step_x) > 1e-5 or abs(step_y) > 1e-5 or abs(step_z) > 1e-5 or abs(step_yaw) > 1e-5)
            if has_drift:
                try:
                    tf = self.tf_buffer.lookup_transform(
                        self.odom_frame,
                        self.base_frame,
                        rclpy.time.Time(),
                        timeout=Duration(seconds=0.0)
                    )
                    ox = tf.transform.translation.x
                    oy = tf.transform.translation.y
                    oz = tf.transform.translation.z
                    _, _, oyaw = euler_from_quaternion(tf.transform.rotation)

                    # Current robot in map
                    cos_curr = math.cos(self.offset_yaw)
                    sin_curr = math.sin(self.offset_yaw)
                    curr_rx = self.offset_x + (ox * cos_curr - oy * sin_curr)
                    curr_ry = self.offset_y + (ox * sin_curr + oy * cos_curr)
                    curr_rz = self.offset_z + oz
                    curr_ryaw = wrap_to_pi(oyaw + self.offset_yaw)

                    # Target robot in map after this step:
                    new_rx = curr_rx + step_x
                    new_ry = curr_ry + step_y
                    new_rz = curr_rz + step_z
                    new_ryaw = wrap_to_pi(curr_ryaw + step_yaw)

                    # Deduct absorbed step from remaining drift
                    self.drift_err_x -= step_x
                    self.drift_err_y -= step_y
                    self.drift_err_z -= step_z
                    self.drift_err_yaw = wrap_to_pi(self.drift_err_yaw - step_yaw)

                    # Update map -> odom transform to produce EXACTLY new_rx, new_ry, new_rz, new_ryaw
                    new_offset_yaw = wrap_to_pi(new_ryaw - oyaw)
                    cos_new = math.cos(new_offset_yaw)
                    sin_new = math.sin(new_offset_yaw)
                    self.offset_yaw = new_offset_yaw
                    self.offset_x = new_rx - (ox * cos_new - oy * sin_new)
                    self.offset_y = new_ry - (ox * sin_new + oy * cos_new)
                    self.offset_z = new_rz - oz
                except Exception:
                    pass

        # Broadcast map -> odom transform
        t = TransformStamped()
        t.header.stamp = now
        t.header.frame_id = self.map_frame
        t.child_frame_id = self.odom_frame
        t.transform.translation.x = float(self.offset_x)
        t.transform.translation.y = float(self.offset_y)
        t.transform.translation.z = float(self.offset_z)
        t.transform.rotation = quaternion_from_yaw(self.offset_yaw)
        self.tf_broadcaster.sendTransform(t)

        # Publish /odometry/global if subscribed
        if self.last_odom_msg is not None and self.global_odom_pub.get_subscription_count() > 0:
            g_odom = Odometry()
            g_odom.header.stamp = now
            g_odom.header.frame_id = self.map_frame
            g_odom.child_frame_id = self.base_frame

            cos_y = math.cos(self.offset_yaw)
            sin_y = math.sin(self.offset_yaw)
            px = self.last_odom_msg.pose.pose.position.x
            py = self.last_odom_msg.pose.pose.position.y
            pz = self.last_odom_msg.pose.pose.position.z

            g_odom.pose.pose.position.x = float(self.offset_x + (px * cos_y - py * sin_y))
            g_odom.pose.pose.position.y = float(self.offset_y + (px * sin_y + py * cos_y))
            g_odom.pose.pose.position.z = float(self.offset_z + pz)

            q_odom = self.last_odom_msg.pose.pose.orientation
            r, p, y = euler_from_quaternion(q_odom)
            y_map = y + self.offset_yaw
            cy = math.cos(y_map * 0.5)
            sy = math.sin(y_map * 0.5)
            cp = math.cos(p * 0.5)
            sp = math.sin(p * 0.5)
            cr = math.cos(r * 0.5)
            sr = math.sin(r * 0.5)

            g_odom.pose.pose.orientation.w = cr * cp * cy + sr * sp * sy
            g_odom.pose.pose.orientation.x = sr * cp * cy - cr * sp * sy
            g_odom.pose.pose.orientation.y = cr * sp * cy + sr * cp * sy
            g_odom.pose.pose.orientation.z = cr * cp * sy - sr * sp * cy

            g_odom.twist = self.last_odom_msg.twist
            g_odom.pose.covariance = self.last_odom_msg.pose.covariance
            g_odom.pose.covariance[0] = 0.04
            g_odom.pose.covariance[7] = 0.04
            g_odom.pose.covariance[14] = 0.04

            self.global_odom_pub.publish(g_odom)


def main(args=None):
    rclpy.init(args=args)
    node = Global3DCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
