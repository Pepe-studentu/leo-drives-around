#!/usr/bin/env python3
"""Rover Kinematic Extended Kalman Filter (Local 6-DOF Odometry).

Fuses:
1. Rover Kinematic Terramechanics Predictor (50 Hz):
   - /cmd_vel (nominal drive and turn commands)
   - /imu/data (attitude tilt: roll, pitch; and gyro yaw rate: omega_z)
   - Incline-induced slip rules (longitudinal uphill/downhill slip and lateral side-slip)
   - Non-Holonomic Constraints (v_z^body ≈ 0 surface contact; v_x^body = 0 during pure spins)
2. LiDAR Front-End Odometry (10 Hz):
   - /kiss/odometry (scan-matching updates)
   - Adaptive innovation gating and covariance weighting

Publishes:
- odom -> base_footprint transform (continuous, jump-free, bounded to terrain)
- /odometry/local (nav_msgs/Odometry)
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, PointCloud2
from sensor_msgs_py import point_cloud2
import tf2_ros


def wrap_angle(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def quat_to_euler(q):
    """Convert geometry_msgs/Quaternion or tuple (w, x, y, z) to (roll, pitch, yaw)."""
    if hasattr(q, 'w'):
        w, x, y, z = q.w, q.x, q.y, q.z
    else:
        w, x, y, z = q[0], q[1], q[2], q[3]

    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def euler_to_quat(roll: float, pitch: float, yaw: float):
    """Convert (roll, pitch, yaw) to (x, y, z, w)."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


def rotation_matrix_rpy(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """3D Rotation Matrix R_z(yaw) * R_y(pitch) * R_x(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    R = np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                cp * cr               ]
    ])
    return R


class RoverKinematicEKF(Node):
    def __init__(self):
        super().__init__('rover_kinematic_ekf')

        # Declare parameters
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_rate', 60.0)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('wheel_odom_topic', '/odom')
        self.declare_parameter('imu_topic', '/imu/data')
        self.declare_parameter('lidar_odom_topic', '/kiss/odometry')

        # Terramechanics parameters
        self.declare_parameter('enable_lidar_update', True)   # Enable adaptive LiDAR fusion
        self.declare_parameter('k_long_slip', 0.03)   # Longitudinal slip per m/s^2 gravity tangent
        self.declare_parameter('k_lat_slip', 0.035)    # Lateral side-slip per m/s^2 gravity tangent
        self.declare_parameter('k_spin_boost', 1.4)    # Lateral slip multiplier when skid-steering
        self.declare_parameter('cmd_timeout', 0.0)      # Timeout to zero commanded velocity (s, 0.0 = disabled)

        # Observability & Option A Parameters
        self.declare_parameter('gamma_observability', 15.0)  # Eigenvalue sensitivity
        self.declare_parameter('r_base_xy', 0.04)           # Base covariance on X/Y (0.20 m std)
        self.declare_parameter('r_base_z', 0.0005)          # Base covariance on Z (~0.02 m std on dense ground)
        self.declare_parameter('r_base_yaw', 0.0076)        # Base covariance on yaw (5 deg std)
        self.declare_parameter('points_topic', '/points')

        # Retrieve parameters
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.enable_lidar_update = self.get_parameter('enable_lidar_update').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        wheel_odom_topic = self.get_parameter('wheel_odom_topic').value
        imu_topic = self.get_parameter('imu_topic').value
        lidar_odom_topic = self.get_parameter('lidar_odom_topic').value
        points_topic = self.get_parameter('points_topic').value

        self.k_long_slip = self.get_parameter('k_long_slip').value
        self.k_lat_slip = self.get_parameter('k_lat_slip').value
        self.k_spin_boost = self.get_parameter('k_spin_boost').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        self.gamma_obs = self.get_parameter('gamma_observability').value
        self.r_base_xy = self.get_parameter('r_base_xy').value
        self.r_base_z = self.get_parameter('r_base_z').value
        self.r_base_yaw = self.get_parameter('r_base_yaw').value

        # Initial eigenvalues: nominal flat ground (low x/y, high z)
        self.current_lambda = (0.4, 0.4, 1000.0)

        # EKF State vector: [x, y, z, yaw] (4x1)
        # Roll and pitch are directly maintained from IMU tilt
        self.state = np.zeros(4)
        # Covariance matrix P (4x4)
        self.P = np.diag([0.01, 0.01, 0.01, 0.005])

        # Process noise Q (per second): Z given 0.04 to dynamically absorb terrain elevation changes
        self.Q_base = np.diag([0.04, 0.04, 0.04, 0.005])

        # Current sensor data
        self.cmd_v = 0.0
        self.cmd_w = 0.0
        self.last_cmd_stamp = None
        self.wheel_vx = 0.0
        self.has_wheel_odom = False

        self.imu_roll = 0.0
        self.imu_pitch = 0.0
        self.imu_ang_vel_x = 0.0
        self.imu_ang_vel_y = 0.0
        self.imu_ang_vel_z = 0.0
        self.imu_yaw_rate = 0.0
        self.last_imu_stamp = None
        self.has_imu = False

        # Velocity tracking for latency compensation and delta updates
        self.last_v_odom = np.zeros(3)
        self.last_v_body = np.zeros(3)
        self.last_gyro_z = 0.0

        # Continuous Innovation Blending (Smooth 60 Hz absorption, zero step jumps)
        self.e_rem = np.zeros(4)

        # LiDAR odometry tracking
        self.last_lidar_msg = None
        self.last_lidar_time = None
        self.last_lidar_meas = None
        self.last_lidar_stamp = None
        self.init_lidar_offset = None

        # Time tracking
        self.last_predict_time = None

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.cmd_sub = self.create_subscription(Twist, cmd_vel_topic, self.cmd_cb, 10)
        self.wheel_sub = self.create_subscription(Odometry, wheel_odom_topic, self.wheel_cb, 10)
        self.imu_sub = self.create_subscription(Imu, imu_topic, self.imu_cb, sensor_qos)
        self.lidar_sub = self.create_subscription(Odometry, lidar_odom_topic, self.lidar_cb, 10)
        self.points_sub = self.create_subscription(PointCloud2, points_topic, self.points_cb, sensor_qos)

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/odometry/local', 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # Timer for prediction and publishing
        dt = 1.0 / max(self.publish_rate, 10.0)
        self.timer = self.create_timer(dt, self.timer_cb)

        self.get_logger().info(
            f"Rover Kinematic EKF initialized at {self.publish_rate:.1f} Hz. "
            f"Frames: {self.odom_frame} -> {self.base_frame}, Wheel Odom: '{wheel_odom_topic}', Slip params: k_long={self.k_long_slip}, k_lat={self.k_lat_slip}"
        )

    def wheel_cb(self, msg: Odometry):
        self.wheel_vx = msg.twist.twist.linear.x
        self.has_wheel_odom = True

    def cmd_cb(self, msg: Twist):
        self.cmd_v = msg.linear.x
        self.cmd_w = msg.angular.z
        self.last_cmd_stamp = self.get_clock().now()

    def imu_cb(self, msg: Imu):
        r, p, y = quat_to_euler(msg.orientation)
        self.imu_roll = r
        self.imu_pitch = p
        # [TEST 3: 3D tilt compensation OFF - flat IMU gyro]
        self.imu_yaw_rate = msg.angular_velocity.z
        self.has_imu = True
        self.last_imu_stamp = msg.header.stamp

    def points_cb(self, msg: PointCloud2):
        """Estimate 3D surface normals and Fisher Information eigenvalues (10 Hz)."""
        try:
            xyz = point_cloud2.read_points_numpy(msg, field_names=["x", "y", "z"], skip_nans=True)
        except Exception:
            return

        if xyz.shape[0] < 50:
            return

        x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
        r2 = x * x + y * y
        valid = (r2 > 0.35**2) & (r2 < 12.0**2)
        x, y, z = x[valid], y[valid], z[valid]
        if len(x) < 50:
            return

        res = 0.4
        max_r = 12.0
        n_bins = int(2.0 * max_r / res)
        gx = np.clip(((x + max_r) / res).astype(int), 0, n_bins - 1)
        gy = np.clip(((y + max_r) / res).astype(int), 0, n_bins - 1)
        idx = gy * n_bins + gx

        counts = np.bincount(idx, minlength=n_bins * n_bins)
        valid_cells = counts >= 2
        z_sum = np.bincount(idx, weights=z, minlength=n_bins * n_bins)
        z_grid = np.zeros(n_bins * n_bins, dtype=np.float32)
        z_grid[valid_cells] = z_sum[valid_cells] / counts[valid_cells]

        H_grid = z_grid.reshape((n_bins, n_bins))
        mask = valid_cells.reshape((n_bins, n_bins))

        dz_dy, dz_dx = np.gradient(H_grid, res)
        valid_grad = mask[1:-1, 1:-1] & mask[1:-1, 2:] & mask[2:, 1:-1]
        gx_vals = dz_dx[1:-1, 1:-1][valid_grad]
        gy_vals = dz_dy[1:-1, 1:-1][valid_grad]

        if len(gx_vals) < 10:
            return

        norm = np.sqrt(gx_vals**2 + gy_vals**2 + 1.0)
        nx = -gx_vals / norm
        ny = -gy_vals / norm
        nz = 1.0 / norm

        lam_x = float(np.sum(nx**2))
        lam_y = float(np.sum(ny**2))
        lam_z = float(np.sum(nz**2))

        self.current_lambda = (max(lam_x, 0.01), max(lam_y, 0.01), max(lam_z, 1.0))

    def lidar_cb(self, msg: Odometry):
        """Adaptive Subspace LiDAR measurement update (Option A: Inverse-Lambda)."""
        if not self.enable_lidar_update:
            return

        now = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
        dt_latency = (now - msg_time).nanoseconds * 1e-9

        # Extract LiDAR pose
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        r, p, yaw = quat_to_euler(ori)
        z_meas = np.array([pos.x, pos.y, pos.z, yaw])

        # First LiDAR measurement initializes coordinate datum
        if self.init_lidar_offset is None or self.last_lidar_meas is None:
            self.init_lidar_offset = z_meas.copy()
            self.last_lidar_meas = z_meas.copy()
            self.last_lidar_stamp = msg_time
            self.state = z_meas.copy()
            self.e_rem = np.zeros(4)
            self.get_logger().info(f"LiDAR datum established: ({pos.x:.2f}, {pos.y:.2f}, {pos.z:.2f}), yaw={math.degrees(yaw):.1f}°")
            self.last_lidar_time = now
            return

        # Time elapsed between LiDAR scans
        dt_lidar = (msg_time - self.last_lidar_stamp).nanoseconds * 1e-9
        if dt_lidar <= 0.0 or dt_lidar > 0.5:
            self.last_lidar_meas = z_meas.copy()
            self.last_lidar_stamp = msg_time
            return

        # Standstill veto: if rover is parked, static friction guarantees zero motion
        is_cmd = (abs(self.cmd_v) > 0.01 or abs(self.cmd_w) > 0.03)
        is_rot = (abs(self.imu_yaw_rate) > 0.02)
        slope = math.hypot(self.imu_pitch, self.imu_roll)
        if not is_cmd and not is_rot and slope <= 0.35:
            self.last_lidar_meas = z_meas.copy()
            self.last_lidar_stamp = msg_time
            return

        # 1. Delta displacement measured by LiDAR between scans:
        dx_lidar = pos.x - self.last_lidar_meas[0]
        dy_lidar = pos.y - self.last_lidar_meas[1]
        last_yaw = self.last_lidar_meas[3]
        self.last_lidar_meas = z_meas.copy()
        self.last_lidar_stamp = msg_time

        # Transform inter-frame displacement into rover body frame at scan k-1:
        c_ly = math.cos(last_yaw)
        s_ly = math.sin(last_yaw)
        delta_x_body_lidar = c_ly * dx_lidar + s_ly * dy_lidar
        delta_y_body_lidar = -s_ly * dx_lidar + c_ly * dy_lidar

        # 2. Predicted body displacement from wheel odometry + terramechanics slip over dt_lidar:
        delta_x_body_pred = self.last_v_body[0] * dt_lidar
        delta_y_body_pred = self.last_v_body[1] * dt_lidar

        # 3. Body-frame innovation (inter-frame step discrepancy between LiDAR and wheels):
        y_body_x = delta_x_body_lidar - delta_x_body_pred
        y_body_y = delta_y_body_lidar - delta_y_body_pred

        # 4. Rotate planar innovation into current EKF odom frame:
        psi = self.state[3]
        c_psi = math.cos(psi)
        s_psi = math.sin(psi)
        y_innov_x = c_psi * y_body_x - s_psi * y_body_y
        y_innov_y = s_psi * y_body_x + c_psi * y_body_y

        # 5. Z Innovation: tracks elevation relative to the terrain surface
        z_aligned_val = pos.z
        if 0.0 < dt_latency < 0.5:
            z_aligned_val += self.last_v_odom[2] * dt_latency
        current_est_z = self.state[2] + self.e_rem[2]
        y_innov_z = z_aligned_val - current_est_z

        y_innov = np.array([y_innov_x, y_innov_y, y_innov_z, 0.0])

        # Innovation gating (reject wild outliers/teleportation)
        trans_err = np.linalg.norm(y_innov[:3])
        if trans_err > 1.50:
            self.get_logger().warn(f"LiDAR innovation rejected: jump={trans_err:.2f}m")
            return

        # Geometric Observability Normalized Weighting:
        # lambda_i sums represent normal distributions across terrain patches.
        lam_x, lam_y, lam_z = self.current_lambda
        lam_tot = max(lam_x + lam_y + lam_z, 1.0)
        norm_lam_x = max(lam_x / lam_tot, 0.01)
        norm_lam_y = max(lam_y / lam_tot, 0.01)
        norm_lam_z = max(lam_z / lam_tot, 0.20)

        # Dynamic measurement covariance:
        # High observability along axis -> low R -> EKF trusts LiDAR
        # Low observability along axis -> high R -> EKF trusts kinematics/wheels
        R_x_body = self.r_base_xy * (0.05 / norm_lam_x)
        R_y_body = self.r_base_xy * (0.05 / norm_lam_y)
        R_z = self.r_base_z * (0.85 / norm_lam_z)

        # Aggressive Gyro-Priority during Rotation:
        # Inflate translation covariance up to 50x during rotation so slip cannot distort X/Y
        rot_speed = max(abs(self.imu_yaw_rate), abs(self.cmd_w))
        if rot_speed > 0.10:
            factor = min(1.0, (rot_speed - 0.10) / 0.30)
            R_x_body *= (1.0 + 49.0 * factor)
            R_y_body *= (1.0 + 49.0 * factor)

        R_yaw = 1e6  # Gyro maintains 100% authority; muted in Kalman update

        # Rotate body-frame planar covariance into odom frame using rover heading
        psi = self.state[3]
        c, s = math.cos(psi), math.sin(psi)
        R_rot = np.array([[c, -s], [s, c]])
        R_body_2d = np.diag([R_x_body, R_y_body])
        R_odom_2d = R_rot @ R_body_2d @ R_rot.T

        # Full 4x4 measurement covariance matrix R
        R = np.zeros((4, 4))
        R[:2, :2] = R_odom_2d
        R[2, 2] = R_z
        R[3, 3] = R_yaw

        # Kalman Measurement Update:
        S = self.P + R
        try:
            K = self.P @ np.linalg.inv(S)
        except np.linalg.LinAlgError:
            return

        # Continuous Innovation Update: queue correction into e_rem for smooth 60 Hz absorption
        corr = K @ y_innov
        corr[3] = 0.0  # Gyroscope has 100% authority over heading in local odometry
        self.e_rem[:3] += corr[:3]
        self.P = (np.eye(4) - K) @ self.P

        self.last_lidar_time = now

        # Periodic diagnostic logging (every 1.0s at 10 Hz)
        if not hasattr(self, '_log_counter'):
            self._log_counter = 0
        self._log_counter += 1
        if self._log_counter % 10 == 0:
            is_spin = (abs(self.cmd_v) < 0.05 and rot_speed > 0.10)
            is_turn_drive = (abs(self.cmd_v) >= 0.05 and rot_speed > 0.10)
            is_fwd = (abs(self.cmd_v) >= 0.05 and rot_speed <= 0.10)
            mode = "SPIN" if is_spin else ("TURN_DRV" if is_turn_drive else ("STRAIGHT" if is_fwd else "STOP"))

            c_psi = math.cos(psi)
            s_psi = math.sin(psi)
            e_fwd = y_innov[0] * c_psi + y_innov[1] * s_psi
            e_lat = -y_innov[0] * s_psi + y_innov[1] * c_psi
            c_fwd = corr[0] * c_psi + corr[1] * s_psi
            c_lat = -corr[0] * s_psi + corr[1] * c_psi

            pct_x = int(norm_lam_x * 100)
            pct_y = int(norm_lam_y * 100)
            pct_z = int(norm_lam_z * 100)
            gyro_deg_s = math.degrees(self.imu_yaw_rate)

            self.get_logger().info(
                f"[EKF {mode:8s}] cmd=(v={self.cmd_v:+.2f}, w={self.cmd_w:+.2f}) wheel_v={self.wheel_vx:+.2f}m/s | "
                f"e_step=(fwd={e_fwd*100:+.1f}cm, lat={e_lat*100:+.1f}cm, z={y_innov[2]:+.2f}m) | "
                f"λ%=({pct_x}%, {pct_y}%, {pct_z}%) | "
                f"Gain=(xy={K[0,0]:.3f}, z={K[2,2]:.3f}) | "
                f"corr=(fwd={c_fwd*100:+.1f}cm, lat={c_lat*100:+.1f}cm)"
            )

    def timer_cb(self):
        """Prediction step and state broadcast (50 Hz)."""
        now = self.get_clock().now()
        if self.last_predict_time is None:
            self.last_predict_time = now
            return

        dt = (now - self.last_predict_time).nanoseconds * 1e-9
        self.last_predict_time = now

        if dt <= 0.0 or dt > 0.5:
            return

        # Check command timeout if enabled (> 0)
        v_cmd = self.cmd_v
        w_cmd = self.cmd_w
        if self.cmd_timeout > 0.0 and self.last_cmd_stamp is not None:
            cmd_age = (now - self.last_cmd_stamp).nanoseconds * 1e-9
            if cmd_age > self.cmd_timeout:
                v_cmd = 0.0
                w_cmd = 0.0

        # 1. Terramechanics Kinematic Predictor:
        g = 9.81
        roll = self.imu_roll
        pitch = self.imu_pitch
        yaw = self.state[3]
        gyro_z = self.imu_yaw_rate

        # Base forward velocity: use real measured wheel joint speeds if available, else fall back to cmd_v
        v_base = self.wheel_vx if self.has_wheel_odom else v_cmd

        is_commanded = (abs(v_base) > 0.01 or abs(v_cmd) > 0.01 or abs(w_cmd) > 0.03)
        is_rotating = (abs(gyro_z) > 0.02)  # Physical rotation detected by IMU gyro (> 1.1 deg/s)
        slope_angle = math.hypot(pitch, roll)

        # Static friction holds rover when stopped unless commanded, physically rotating, or slope is steep (>20 deg / 0.35 rad)
        if is_commanded or is_rotating or slope_angle > 0.35:
            # Longitudinal slip: pitch < 0 is nose up, gravity opposes forward speed
            v_slip_long = self.k_long_slip * g * math.sin(pitch)
            # Lateral side-slip: roll > 0 tilts left side up, gravity pulls right (-y)
            spin_factor = 1.0 + self.k_spin_boost * abs(gyro_z) if abs(gyro_z) > 0.1 else 1.0
            v_slip_lat = - self.k_lat_slip * g * math.sin(roll) * spin_factor

            # Diff-drive forward vs spin in place
            if abs(v_base) < 0.01 and (abs(w_cmd) > 0.05 or is_rotating):
                v_x_body = v_slip_long
            else:
                v_x_body = v_base + v_slip_long
            v_y_body = v_slip_lat
        else:
            v_x_body = 0.0
            v_y_body = 0.0
            gyro_z = 0.0

        v_z_body = 0.0

        # 2. Transform Body Velocities to Odom Frame:
        R_mat = rotation_matrix_rpy(roll, pitch, yaw)
        v_body = np.array([v_x_body, v_y_body, v_z_body])
        self.last_v_body = v_body.copy()
        v_odom = R_mat @ v_body

        # Track velocity for latency compensation
        self.last_v_odom = v_odom.copy()
        self.last_gyro_z = gyro_z

        # 3. Propagate State:
        self.state[:3] += v_odom * dt
        # Yaw rate: blend IMU gyro with turning command
        self.state[3] += gyro_z * dt
        self.state[3] = wrap_angle(self.state[3])

        # Smoothly absorb LiDAR innovation into state at 60 Hz (C1 continuous trajectory)
        blend_factor = 1.0 - math.exp(-15.0 * dt)
        step_corr = self.e_rem[:3] * blend_factor
        step_yaw = wrap_angle(self.e_rem[3]) * blend_factor
        self.state[:3] += step_corr
        self.state[3] = wrap_angle(self.state[3] + step_yaw)
        self.e_rem[:3] -= step_corr
        self.e_rem[3] = wrap_angle(self.e_rem[3] - step_yaw)

        # 4. Propagate Covariance:
        # P = P + Q * dt
        Q = self.Q_base.copy()
        # On slopes, increase process uncertainty slightly
        if abs(pitch) > 0.1 or abs(roll) > 0.1:
            Q[0, 0] *= 1.5
            Q[1, 1] *= 1.5
        self.P += Q * dt

        # 5. Broadcast Odometry & TF
        self.broadcast_odometry(now, v_body, v_odom)

    def broadcast_odometry(self, now, v_body, v_odom):
        """Publish ROS Odometry message and TF."""
        roll = self.imu_roll
        pitch = self.imu_pitch
        # Broadcast single source of truth pose directly
        pub_pos = self.state[:3]
        pub_yaw = self.state[3]
        qx, qy, qz, qw = euler_to_quat(roll, pitch, pub_yaw)

        # Broadcast TF
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = float(pub_pos[0])
            t.transform.translation.y = float(pub_pos[1])
            t.transform.translation.z = float(pub_pos[2])
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

        # Publish Odometry msg
        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = float(pub_pos[0])
        odom.pose.pose.position.y = float(pub_pos[1])
        odom.pose.pose.position.z = float(pub_pos[2])
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # 6x6 Pose Covariance
        odom.pose.covariance[0] = float(self.P[0, 0])
        odom.pose.covariance[7] = float(self.P[1, 1])
        odom.pose.covariance[14] = float(self.P[2, 2])
        odom.pose.covariance[21] = 0.001  # roll from IMU
        odom.pose.covariance[28] = 0.001  # pitch from IMU
        odom.pose.covariance[35] = float(self.P[3, 3])

        # Velocities in child_frame_id (base_footprint)
        odom.twist.twist.linear.x = float(v_body[0])
        odom.twist.twist.linear.y = float(v_body[1])
        odom.twist.twist.linear.z = float(v_body[2])
        odom.twist.twist.angular.z = float(self.imu_yaw_rate)

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = RoverKinematicEKF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
