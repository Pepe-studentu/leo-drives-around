#!/usr/bin/env python3
"""Bayesian Confidence-Weighted 3D Height-Map and Traversability Costmap Node.

1. Bins raw LiDAR scans into a 2.5D elevation grid in the global map frame
   and publishes the accumulated terrain as a PointCloud2 (/height_map/cloud).
2. Computes terrain traversability costmaps (slope, step, and proximity shadow occlusions)
   and publishes nav_msgs/OccupancyGrid on /traversability/costmap for Nav2.

Update Logic:
- Each cell (r, c) maintains estimated height (h) and estimation variance (var).
- Observation variance per point:
    sigma_obs^2 = sigma_sensor^2 + (d * sigma_angular)^2 + k_drift * dt_oracle
- Close-range observations naturally dominate and lock in terrain geometry against distant noisy scans.
- Slope cost:
    slope <= 20 deg: cost = 0
    20 deg < slope <= 30 deg: cost ramps linearly from 0 to 85
    slope > 30 deg: cost = 100 (LETHAL)
- Step cost:
    step <= 5cm: cost = 0
    5cm < step <= 12cm: cost ramps linearly from 0 to 80
    step > 12cm: cost = 100 (LETHAL)
"""
import cv2
import numpy as np
import rclpy
import rclpy.time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid, Odometry
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

        # Grid parameters
        self.declare_parameter('input_cloud_topic', '/points')
        self.declare_parameter('oracle_topic', '/satellite/oracle_fix')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('resolution', 0.10)
        self.declare_parameter('map_size_x', 50.0)
        self.declare_parameter('map_size_y', 50.0)
        self.declare_parameter('origin_x', -25.0)
        self.declare_parameter('origin_y', -25.0)
        self.declare_parameter('min_range', 0.35)
        self.declare_parameter('max_range', 15.0)
        self.declare_parameter('update_period', 0.50)

        # Bayesian Confidence Tuning Knobs
        self.declare_parameter('sigma_sensor', 0.02)      # Base LiDAR depth noise std dev (m)
        self.declare_parameter('sigma_angular', 0.01)     # Orientation lever-arm uncertainty rate (rad)
        self.declare_parameter('k_drift', 0.005)          # Drift variance growth rate (m^2/s) between oracle fixes
        self.declare_parameter('min_variance', 0.0001)    # Variance floor (m^2) to keep filter responsive
        self.declare_parameter('mahalanobis_gate', 3.5)   # Outlier rejection threshold in std devs

        # Traversability Costmap Knobs (Pure Slope-Based)
        self.declare_parameter('slope_free_deg', 30.0)
        self.declare_parameter('slope_warn_deg', 35.0)
        self.declare_parameter('slope_lethal_deg', 45.0)
        self.declare_parameter('slope_warn_cost', 10.0)
        self.declare_parameter('step_free_m', 0.05)
        self.declare_parameter('step_lethal_m', 0.12)
        self.declare_parameter('lidar_deadzone_radius', 1.05)
        self.declare_parameter('rover_footprint_radius', 0.45)
        self.declare_parameter('publish_debug_costmaps', True)

        def _str(name):
            return self.get_parameter(name).get_parameter_value().string_value

        def _float(name):
            return self.get_parameter(name).get_parameter_value().double_value

        self.input_cloud_topic = _str('input_cloud_topic')
        self.oracle_topic = _str('oracle_topic')
        self.map_frame = _str('map_frame')
        self.resolution = _float('resolution')
        self.map_size_x = _float('map_size_x')
        self.map_size_y = _float('map_size_y')
        self.origin_x = _float('origin_x')
        self.origin_y = _float('origin_y')
        self.min_range = _float('min_range')
        self.max_range = _float('max_range')
        self.update_period = _float('update_period')

        self.sigma_sensor = _float('sigma_sensor')
        self.sigma_angular = _float('sigma_angular')
        self.k_drift = _float('k_drift')
        self.min_variance = _float('min_variance')
        self.mahalanobis_gate = _float('mahalanobis_gate')

        self.slope_free_deg = _float('slope_free_deg')
        self.slope_warn_deg = _float('slope_warn_deg')
        self.slope_lethal_deg = _float('slope_lethal_deg')
        self.slope_warn_cost = _float('slope_warn_cost')
        self.step_free_m = _float('step_free_m')
        self.step_lethal_m = _float('step_lethal_m')
        self.lidar_deadzone_radius = _float('lidar_deadzone_radius')
        self.rover_footprint_radius = _float('rover_footprint_radius')
        self.publish_debug_costmaps = bool(self.get_parameter('publish_debug_costmaps').value)
        self.spawn_bootstrapped = False

        # Grid representation: 2.5D height + variance
        self.W = int(round(self.map_size_x / self.resolution))
        self.H = int(round(self.map_size_y / self.resolution))
        self.height = np.full((self.H, self.W), -np.inf, dtype=np.float32)
        self.var = np.full((self.H, self.W), np.inf, dtype=np.float32)
        self.hits = np.zeros((self.H, self.W), dtype=np.int32)

        # Precompute coordinate grids for fast distance vectorization
        self.grid_x = self.origin_x + (np.arange(self.W, dtype=np.float32) + 0.5) * self.resolution
        self.grid_y = self.origin_y + (np.arange(self.H, dtype=np.float32) + 0.5) * self.resolution

        # Oracle fix tracking
        self.last_oracle_stamp = None
        self.last_stamp = None

        # TF2 listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.cb_cloud = MutuallyExclusiveCallbackGroup()
        self.cb_timer = MutuallyExclusiveCallbackGroup()

        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT

        # Subscriptions
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.input_cloud_topic, self.cloud_cb, sensor_qos,
            callback_group=self.cb_cloud)

        self.oracle_sub = self.create_subscription(
            Odometry, self.oracle_topic, self.oracle_cb, QoSProfile(depth=10))

        # Publishers
        self.raw_cloud_pub = self.create_publisher(
            PointCloud2, '/height_map/raw_cloud', QoSProfile(depth=1))
        self.height_cloud_pub = self.create_publisher(
            PointCloud2, '/height_map/cloud', QoSProfile(depth=1))

        # Costmap Publishers
        self.costmap_pub = self.create_publisher(
            OccupancyGrid, '/traversability/costmap', QoSProfile(depth=1))
        self.slope_costmap_pub = self.create_publisher(
            OccupancyGrid, '/traversability/slope_costmap', QoSProfile(depth=1))
        self.step_costmap_pub = self.create_publisher(
            OccupancyGrid, '/traversability/step_costmap', QoSProfile(depth=1))

        # Periodic map publisher for RViz / Nav2
        self.timer = self.create_timer(
            self.update_period, self.on_timer, callback_group=self.cb_timer)

        self.get_logger().info(
            f"Traversability & Height Map initialized: {self.W}x{self.H} cells @ {self.resolution:.2f}m. "
            f"Pure slope bands: [<= {self.slope_free_deg:.0f}°: cost 0 | "
            f"{self.slope_free_deg:.0f}°-{self.slope_warn_deg:.0f}°: cost {self.slope_warn_cost:.0f} | "
            f"{self.slope_warn_deg:.0f}°-{self.slope_lethal_deg:.0f}°: linear ramp to 100 | "
            f"> {self.slope_lethal_deg:.0f}°: lethal], LiDAR dead-zone radius: {self.lidar_deadzone_radius:.2f}m"
        )

    def oracle_cb(self, msg: Odometry):
        """Track global satellite oracle fix timestamps for freshness calculation."""
        self.last_oracle_stamp = msg.header.stamp

    def cloud_cb(self, msg: PointCloud2):
        """Transform scan into map frame and update elevation grid using Bayesian Kalman updates."""
        xyz = point_cloud2.read_points_numpy(
            msg, field_names=["x", "y", "z"], skip_nans=True)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if xyz.shape[0] == 0:
            return

        ranges = np.linalg.norm(xyz, axis=1)
        valid_range = (ranges >= self.min_range) & (ranges <= self.max_range)
        xyz = xyz[valid_range]
        ranges = ranges[valid_range]
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
            except TransformException:
                return

        q = tf.transform.rotation
        t = tf.transform.translation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        pts_map = xyz @ R.T + np.array([t.x, t.y, t.z])
        self.last_stamp = msg.header.stamp

        # Bootstrap ground around initial spawn position
        if not self.spawn_bootstrapped:
            dist_spawn2 = (self.grid_x[None, :] - t.x)**2 + (self.grid_y[:, None] - t.y)**2
            spawn_mask = dist_spawn2 <= 1.5**2
            # Ground level under chassis: LiDAR is at z = t.z, wheels on ground at t.z - 0.23m
            self.height[spawn_mask] = float(t.z - 0.23)
            self.var[spawn_mask] = 0.01
            self.hits[spawn_mask] = 10
            self.spawn_bootstrapped = True
            self.get_logger().info(
                f"Bootstrapped spawn ground at ({t.x:.2f}, {t.y:.2f}) with height {t.z - 0.23:.2f}m")

        # Debug: publish raw transformed points if subscribed
        if self.raw_cloud_pub.get_subscription_count() > 0:
            hdr = Header(stamp=msg.header.stamp, frame_id=self.map_frame)
            self.raw_cloud_pub.publish(
                point_cloud2.create_cloud_xyz32(hdr, pts_map.astype(np.float32)))

        # Freshness of global correction (dt_oracle)
        if self.last_oracle_stamp is not None:
            cloud_t = rclpy.time.Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
            oracle_t = rclpy.time.Time.from_msg(self.last_oracle_stamp).nanoseconds * 1e-9
            dt_oracle = max(0.0, min(cloud_t - oracle_t, 3.0))
        else:
            dt_oracle = 1.0

        # Point variance: sensor noise + distance lever arm + global drift growth
        var_drift = self.k_drift * dt_oracle
        pt_var = (self.sigma_sensor ** 2) + (ranges * self.sigma_angular) ** 2 + var_drift

        # Bin points to 2D grid
        xs, ys, zs = pts_map[:, 0], pts_map[:, 1], pts_map[:, 2]
        cols = ((xs - self.origin_x) / self.resolution).astype(np.int32)
        rows = ((ys - self.origin_y) / self.resolution).astype(np.int32)
        in_bounds = (cols >= 0) & (cols < self.W) & (rows >= 0) & (rows < self.H)

        cols = cols[in_bounds]
        rows = rows[in_bounds]
        zs = zs[in_bounds].astype(np.float32)
        pt_var = pt_var[in_bounds].astype(np.float32)

        if rows.size == 0:
            return

        # Vectorized aggregation per cell
        flat_idx = rows * self.W + cols
        total_cells = self.H * self.W

        counts = np.bincount(flat_idx, minlength=total_cells).astype(np.float32)
        valid_cells = counts > 0

        z_sums = np.bincount(flat_idx, weights=zs, minlength=total_cells).astype(np.float32)
        z_obs = np.zeros(total_cells, dtype=np.float32)
        z_obs[valid_cells] = z_sums[valid_cells] / counts[valid_cells]

        var_sums = np.bincount(flat_idx, weights=pt_var, minlength=total_cells).astype(np.float32)
        var_obs = np.full(total_cells, np.inf, dtype=np.float32)
        # Variance scales inversely with point count within the cell
        var_obs[valid_cells] = (var_sums[valid_cells] / counts[valid_cells]) / np.sqrt(counts[valid_cells])

        # Reshape to grid
        valid_mask = valid_cells.reshape((self.H, self.W))
        z_obs_grid = z_obs.reshape((self.H, self.W))
        var_obs_grid = var_obs.reshape((self.H, self.W))

        # Bayesian Kalman Grid Update:
        # 1. Unvisited cells: initialize directly
        new_cells = valid_mask & (self.var == np.inf)
        self.height[new_cells] = z_obs_grid[new_cells]
        self.var[new_cells] = var_obs_grid[new_cells]

        # 2. Visited cells: Kalman fusion with Mahalanobis outlier gating
        old_cells = valid_mask & (self.var < np.inf)
        if np.any(old_cells):
            h_prior = self.height[old_cells]
            v_prior = self.var[old_cells]
            h_meas = z_obs_grid[old_cells]
            v_meas = var_obs_grid[old_cells]

            # Mahalanobis outlier gating: reject transient dust / phantom spikes
            diff = np.abs(h_meas - h_prior)
            sigma_diff = np.sqrt(v_prior + v_meas)
            inlier = (diff <= self.mahalanobis_gate * sigma_diff) | (diff < 0.20)

            # Kalman gain K in [0, 1]
            K = np.zeros_like(h_prior)
            K[inlier] = v_prior[inlier] / (v_prior[inlier] + v_meas[inlier])

            # Update height and variance
            h_updated = h_prior + K * (h_meas - h_prior)
            v_updated = np.maximum(self.min_variance, (1.0 - K) * v_prior)

            # If rejected by gate, slowly increase uncertainty so valid changes can eventually absorb
            outlier = ~inlier
            v_updated[outlier] = np.minimum(1.0, v_prior[outlier] + 0.001)

            self.height[old_cells] = h_updated
            self.var[old_cells] = v_updated

        self.hits[valid_mask] += 1

    def compute_costmaps(self):
        """Compute terrain traversability costmaps from elevation grid.
        1. Morphological sub-footprint gap closing to infill scan lines.
        2. Footprint-scale slope gradient and step height evaluation.
        """
        H, W = self.H, self.W
        res = self.resolution
        valid = np.isfinite(self.height) & (self.var < np.inf) & (self.hits > 0)

        # 0. Check rover position in map frame
        rover_x, rover_y, rover_z = 0.0, 0.0, 0.0
        has_rover_pos = False
        try:
            tf_rover = self.tf_buffer.lookup_transform(
                self.map_frame, 'base_footprint', rclpy.time.Time())
            rover_x = tf_rover.transform.translation.x
            rover_y = tf_rover.transform.translation.y
            rover_z = tf_rover.transform.translation.z
            has_rover_pos = True
        except TransformException:
            pass

        # Spawn Bootstrapping: At initial boot, initialize the blind cone under the rover
        # with the spawn ground elevation so the rover starts on guaranteed explored land.
        if has_rover_pos and not self.spawn_bootstrapped:
            dist2_init = (self.grid_x[None, :] - rover_x)**2 + (self.grid_y[:, None] - rover_y)**2
            spawn_mask = (dist2_init <= self.lidar_deadzone_radius**2)
            self.height[spawn_mask] = rover_z
            self.var[spawn_mask] = self.min_variance
            self.hits[spawn_mask] = 10
            valid[spawn_mask] = True
            self.spawn_bootstrapped = True
            self.get_logger().info(
                f"Spawn bootstrapped: initialized {int(np.sum(spawn_mask))} cells at z={rover_z:.2f}m "
                f"within blind cone ({self.lidar_deadzone_radius:.2f}m)."
            )

        if not np.any(valid):
            empty_cost = np.zeros((H, W), dtype=np.int8)
            empty_debug = np.full((H, W), -1, dtype=np.int8)
            return empty_cost, empty_debug, empty_debug

        # 1. Sub-Footprint Gap Closing
        # A 3x3 structuring element corresponds to 30cm x 30cm (smaller than the rover's 45cm chassis).
        valid_u8 = valid.astype(np.uint8)
        k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        valid_closed = cv2.morphologyEx(valid_u8, cv2.MORPH_CLOSE, k3).astype(bool)

        # Infill height for cells in the closed sub-footprint gaps
        gap_cells = valid_closed & (~valid)
        h_dense = np.where(valid, self.height, 0.0).astype(np.float32)
        if np.any(gap_cells):
            m_valid = valid.astype(np.float32)
            k_box = np.ones((3, 3), dtype=np.float32)
            h_sum = cv2.filter2D(h_dense, -1, k_box)
            m_sum = cv2.filter2D(m_valid, -1, k_box)
            valid_fill = gap_cells & (m_sum > 0)
            h_dense[valid_fill] = h_sum[valid_fill] / m_sum[valid_fill]

        # 2. Footprint-Scale Slope Estimation
        # Smooth elevation locally within the rover footprint to eliminate high-frequency LiDAR jitter
        m_closed = valid_closed.astype(np.float32)
        k_box = np.ones((3, 3), dtype=np.float32)
        h_closed_sum = cv2.filter2D(h_dense, -1, k_box)
        m_closed_sum = cv2.filter2D(m_closed, -1, k_box)
        h_smooth = np.zeros_like(h_dense)
        valid_smooth = valid_closed & (m_closed_sum > 0)
        h_smooth[valid_smooth] = h_closed_sum[valid_smooth] / m_closed_sum[valid_smooth]

        # Central differences on interior slices using valid mask (no boundary cliffs or NaNs)
        h_c = h_smooth[1:-1, 1:-1]
        v_c = valid_closed[1:-1, 1:-1]
        h_r = h_smooth[1:-1, 2:]
        v_r = valid_closed[1:-1, 2:]
        h_l = h_smooth[1:-1, :-2]
        v_l = valid_closed[1:-1, :-2]
        h_d = h_smooth[2:, 1:-1]
        v_d = valid_closed[2:, 1:-1]
        h_u = h_smooth[:-2, 1:-1]
        v_u = valid_closed[:-2, 1:-1]

        dx = np.zeros_like(h_c)
        mask_both_x = v_r & v_l
        dx[mask_both_x] = (h_r[mask_both_x] - h_l[mask_both_x]) / (2.0 * res)
        mask_r_only = v_r & (~v_l) & v_c
        dx[mask_r_only] = (h_r[mask_r_only] - h_c[mask_r_only]) / res
        mask_l_only = v_l & (~v_r) & v_c
        dx[mask_l_only] = (h_c[mask_l_only] - h_l[mask_l_only]) / res

        dy = np.zeros_like(h_c)
        mask_both_y = v_d & v_u
        dy[mask_both_y] = (h_d[mask_both_y] - h_u[mask_both_y]) / (2.0 * res)
        mask_d_only = v_d & (~v_u) & v_c
        dy[mask_d_only] = (h_d[mask_d_only] - h_c[mask_d_only]) / res
        mask_u_only = v_u & (~v_d) & v_c
        dy[mask_u_only] = (h_c[mask_u_only] - h_u[mask_u_only]) / res

        slope_deg_inner = np.degrees(np.arctan(np.sqrt(dx**2 + dy**2)))
        slope_deg = np.zeros((H, W), dtype=np.float32)
        slope_deg[1:-1, 1:-1] = np.where(v_c, slope_deg_inner, 0.0)

        # Cost assignment based strictly on slope
        cost_slope = np.zeros((H, W), dtype=np.float32)

        # Band 2: 30° to 35° -> flat cost of 10.0
        mask_warn = valid_closed & (slope_deg > self.slope_free_deg) & (slope_deg <= self.slope_warn_deg)
        cost_slope[mask_warn] = self.slope_warn_cost

        # Band 3: 35° to 45° -> ramps linearly from 10.0 to 100.0
        mask_ramp = valid_closed & (slope_deg > self.slope_warn_deg) & (slope_deg <= self.slope_lethal_deg)
        ramp_progress = (slope_deg[mask_ramp] - self.slope_warn_deg) / (self.slope_lethal_deg - self.slope_warn_deg)
        cost_slope[mask_ramp] = self.slope_warn_cost + (100.0 - self.slope_warn_cost) * ramp_progress

        # Band 4: > 45° -> 100.0 (Lethal)
        mask_lethal = valid_closed & (slope_deg > self.slope_lethal_deg)
        cost_slope[mask_lethal] = 100.0

        cost_combined = cost_slope.copy()

        # 3. Unexplored terrain is free (0) for global planning (explored progressively by tilted LiDAR)
        cost_combined[~valid_closed] = 0.0
        if has_rover_pos:
            dist2 = (self.grid_x[None, :] - rover_x)**2 + (self.grid_y[:, None] - rover_y)**2
            in_footprint = (dist2 <= self.rover_footprint_radius**2)
            cost_combined[in_footprint] = 0.0

        cost_grid = np.clip(cost_combined, 0.0, 100.0).astype(np.int8)

        # 4. Debug Layers (Blue-Red 1..98)
        slope_layer = np.full((H, W), -1, dtype=np.int8)
        slope_norm = np.clip(slope_deg / 45.0, 0.0, 1.0)
        slope_layer[valid_closed] = np.clip(1 + np.round(slope_norm[valid_closed] * 97.0), 1, 98).astype(np.int8)

        # Step layer (debug)
        step_r = np.where(v_r & v_c, np.abs(h_r - h_c), 0.0)
        step_l = np.where(v_l & v_c, np.abs(h_l - h_c), 0.0)
        step_d = np.where(v_d & v_c, np.abs(h_d - h_c), 0.0)
        step_u = np.where(v_u & v_c, np.abs(h_u - h_c), 0.0)
        step_inner = np.maximum.reduce([step_r, step_l, step_d, step_u])
        step_m = np.zeros((H, W), dtype=np.float32)
        step_m[1:-1, 1:-1] = np.where(v_c, step_inner, 0.0)
        step_norm = np.clip(step_m / 0.30, 0.0, 1.0)
        step_layer = np.full((H, W), -1, dtype=np.int8)
        step_layer[valid_closed] = np.clip(1 + np.round(step_norm[valid_closed] * 97.0), 1, 98).astype(np.int8)

        return cost_grid, slope_layer, step_layer

    def create_occ_msg(self, data_2d, stamp):
        """Create a nav_msgs/OccupancyGrid message."""
        msg = OccupancyGrid()
        msg.header = Header(stamp=stamp, frame_id=self.map_frame)
        msg.info.map_load_time = stamp
        msg.info.resolution = float(self.resolution)
        msg.info.width = int(self.W)
        msg.info.height = int(self.H)
        msg.info.origin.position.x = float(self.origin_x)
        msg.info.origin.position.y = float(self.origin_y)
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = data_2d.ravel().tolist()
        return msg

    def on_timer(self):
        """Publish accumulated height map and traversability costmaps."""
        stamp = self.last_stamp if self.last_stamp is not None else self.get_clock().now().to_msg()

        # 1. Publish 3D PointCloud2 height map (only if subscribed to conserve CPU/RAM)
        if self.height_cloud_pub.get_subscription_count() > 0:
            filled = np.isfinite(self.height) & (self.var < np.inf) & (self.hits > 0)
            rr, cc = np.nonzero(filled)
            if rr.size > 0:
                cx = self.origin_x + (cc.astype(np.float32) + 0.5) * self.resolution
                cy = self.origin_y + (rr.astype(np.float32) + 0.5) * self.resolution
                cz = self.height[rr, cc]

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

        # 2. Compute and publish costmaps (total costmap, physical slope layer, physical step layer)
        cost_grid, slope_layer, step_layer = self.compute_costmaps()
        self.costmap_pub.publish(self.create_occ_msg(cost_grid, stamp))

        if self.publish_debug_costmaps:
            self.slope_costmap_pub.publish(self.create_occ_msg(slope_layer, stamp))
            self.step_costmap_pub.publish(self.create_occ_msg(step_layer, stamp))


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
