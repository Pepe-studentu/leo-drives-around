#!/usr/bin/env python3
"""Elevation / traversability mapping node.

Slice 2c: accumulate the incoming point cloud into a fixed height grid in the
map frame and publish it as an OccupancyGrid for eyeballing in RViz.
"""
import warnings

import numpy as np
import rclpy
import rclpy.time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
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


class TraversabilityNode(Node):

    def __init__(self):
        super().__init__('traversability_node')

        # --- parameters (defaults here; overridden by config/traversability.yaml) ---
        self.declare_parameter('input_cloud_topic', '/points')
        self.declare_parameter('output_costmap_topic', '/traversability/costmap')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('resolution', 0.10)          # metres per cell
        self.declare_parameter('map_size_x', 60.0)          # metres
        self.declare_parameter('map_size_y', 60.0)          # metres
        self.declare_parameter('origin_x', -30.0)           # world coord of grid corner
        self.declare_parameter('origin_y', -30.0)
        self.declare_parameter('stride', 1)                 # keep every Nth point
        self.declare_parameter('max_range', 20.0)           # drop points beyond this (m)
        self.declare_parameter('update_period', 0.25)       # on_timer period (s)
        self.declare_parameter('fusion', 'max')             # 'max' (running max) or 'ema'
        self.declare_parameter('ema_alpha', 0.2)            # only used when fusion == 'ema'
        self.declare_parameter('outlier_gate', 0.0)         # m; 0 disables the gate
        self.declare_parameter('outlier_min_hits', 10)      # only gate well-seen cells
        self.declare_parameter('min_hits', 3)               # cell ignored below this (2e)
        self.declare_parameter('smooth_iters', 0)           # 3x3 blur passes before slope
        self.declare_parameter('publish_debug_layers', False)  # height/slope/step/rough grids
        self.declare_parameter('slope_method', 'planefit')  # 'planefit' or 'gradient'
        self.declare_parameter('slope_min_pts', 4)          # min filled 3x3 neighbours for a fit
        # --- 2f: cost thresholds (tuned in Phase 5) ---
        self.declare_parameter('slope_free_deg', 10.0)      # below this: no penalty
        self.declare_parameter('slope_lethal_deg', 25.0)    # at/above this: cost 100
        self.declare_parameter('step_lethal_m', 0.15)       # hard cut -> cost 100
        self.declare_parameter('rough_free', 0.03)
        self.declare_parameter('rough_lethal', 0.12)

        # Use the typed accessors: .value is `Any | None` and trips static
        # type checkers; .string_value / .double_value are concretely typed.
        def _str(name):
            return self.get_parameter(name).get_parameter_value().string_value

        def _float(name):
            return self.get_parameter(name).get_parameter_value().double_value

        def _int(name):
            return self.get_parameter(name).get_parameter_value().integer_value

        self.input_cloud_topic = _str('input_cloud_topic')
        self.output_costmap_topic = _str('output_costmap_topic')
        self.map_frame = _str('map_frame')
        self.resolution = _float('resolution')
        self.map_size_x = _float('map_size_x')
        self.map_size_y = _float('map_size_y')
        self.origin_x = _float('origin_x')
        self.origin_y = _float('origin_y')
        self.stride = max(1, _int('stride'))
        self.max_range = _float('max_range')
        self.fusion = _str('fusion')
        self.ema_alpha = _float('ema_alpha')
        self.outlier_gate = _float('outlier_gate')
        self.outlier_min_hits = _int('outlier_min_hits')
        self.min_hits = max(1, _int('min_hits'))
        self.smooth_iters = _int('smooth_iters')
        self.publish_debug_layers = bool(
            self.get_parameter('publish_debug_layers').get_parameter_value().bool_value)
        self.slope_method = _str('slope_method')
        self.slope_min_pts = _int('slope_min_pts')
        self.slope_free_deg = _float('slope_free_deg')
        self.slope_lethal_deg = _float('slope_lethal_deg')
        self.step_lethal_m = _float('step_lethal_m')
        self.rough_free = _float('rough_free')
        self.rough_lethal = _float('rough_lethal')

        # --- grid allocation (once) ---
        self.W = int(round(self.map_size_x / self.resolution))
        self.H = int(round(self.map_size_y / self.resolution))
        self.height = np.full((self.H, self.W), -np.inf, dtype=np.float32)
        self.hits = np.zeros((self.H, self.W), dtype=np.int32)

        # latest cloud stashed by cloud_cb, consumed by on_timer
        self.pts_map = None
        self.pts_stamp = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)   # fills the buffer in the background

        # Sensor stream: match the publisher's BEST_EFFORT QoS or the callback never fires.
        # cloud_cb and on_timer get their own callback groups so the TF
        # listener (default group) keeps processing /tf on another executor
        # thread while cloud_cb is blocked inside lookup_transform()'s wait.
        self.cb_cloud = MutuallyExclusiveCallbackGroup()
        self.cb_timer = MutuallyExclusiveCallbackGroup()

        sensor_qos = QoSProfile(depth=5)
        sensor_qos.reliability = ReliabilityPolicy.BEST_EFFORT
        self.cloud_sub = self.create_subscription(
            PointCloud2, self.input_cloud_topic, self.cloud_cb, sensor_qos,
            callback_group=self.cb_cloud)

        # Costmap: latched so late subscribers (Nav2, RViz) get the last map.
        latched = QoSProfile(depth=1)
        latched.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.cost_pub = self.create_publisher(
            OccupancyGrid, self.output_costmap_topic, latched)

        # debug_cloud is cheap and handy -- always on.
        self.debug_cloud_pub = self.create_publisher(
            PointCloud2, '/traversability/debug_cloud', QoSProfile(depth=1))
        # The 4 debug OccupancyGrids cost ~4 extra full-grid serialisations per
        # tick; off by default (publish_debug_layers) for performance.
        if self.publish_debug_layers:
            self.debug_height_pub = self.create_publisher(
                OccupancyGrid, '/traversability/debug_height', latched)
            self.debug_slope_pub = self.create_publisher(
                OccupancyGrid, '/traversability/debug_slope', latched)
            self.debug_step_pub = self.create_publisher(
                OccupancyGrid, '/traversability/debug_step', latched)
            self.debug_rough_pub = self.create_publisher(
                OccupancyGrid, '/traversability/debug_rough', latched)

        self.timer = self.create_timer(
            _float('update_period'), self.on_timer, callback_group=self.cb_timer)

        self.get_logger().info(
            f"traversability_node up: cloud='{self.input_cloud_topic}' "
            f"-> costmap='{self.output_costmap_topic}', frame='{self.map_frame}', "
            f"grid={self.W}x{self.H} @ {self.resolution} m/cell"
        )

    # ------------------------------------------------------------------ #
    def cloud_cb(self, msg: PointCloud2):
        """Cheap: read, downsample, clip, transform to map, stash."""
        age = (self.get_clock().now()
               - rclpy.time.Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
        self.get_logger().info(
            f"cloud: {msg.width * msg.height} pts, frame='{msg.header.frame_id}', "
            f"age={age * 1000:.0f} ms", throttle_duration_sec=1.0)

        xyz = point_cloud2.read_points_numpy(
            msg, field_names=["x", "y", "z"], skip_nans=True)
        # skip_nans does NOT drop +/-inf: ~half of every scan is [inf,-inf,-inf]
        # no-return rays. Strip them explicitly before any arithmetic.
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if xyz.shape[0] == 0:
            return
        xyz = xyz[::self.stride]
        xyz = xyz[np.linalg.norm(xyz, axis=1) < self.max_range]

        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                rclpy.time.Time.from_msg(msg.header.stamp),
                Duration(seconds=0.1),
            )
        except TransformException as e:
            self.get_logger().warn(f"no TF yet: {e}", throttle_duration_sec=2.0)
            return

        q = tf.transform.rotation
        t = tf.transform.translation
        R = quat_to_R(q.x, q.y, q.z, q.w)
        self.pts_map = xyz @ R.T + np.array([t.x, t.y, t.z])   # (N, 3) in map frame
        self.pts_stamp = msg.header.stamp

        hdr = Header(stamp=msg.header.stamp, frame_id=self.map_frame)
        self.debug_cloud_pub.publish(
            point_cloud2.create_cloud_xyz32(hdr, self.pts_map.astype(np.float32)))

    # ------------------------------------------------------------------ #
    def on_timer(self):
        """Heavy work at a fixed rate: bin the stashed points into the grid."""
        if self.pts_map is None:
            return
        pts, stamp = self.pts_map, self.pts_stamp

        xs, ys, zs = pts[:, 0], pts[:, 1], pts[:, 2]
        cols = ((xs - self.origin_x) / self.resolution).astype(np.int32)
        rows = ((ys - self.origin_y) / self.resolution).astype(np.int32)
        m = (cols >= 0) & (cols < self.W) & (rows >= 0) & (rows < self.H)
        rows, cols, zs = rows[m], cols[m], zs[m].astype(np.float32)

        # --- 2d: reduce THIS scan to one height per cell, then fuse ---
        obs = np.full((self.H, self.W), -np.inf, dtype=np.float32)
        np.maximum.at(obs, (rows, cols), zs)          # highest point per cell, this scan
        seen = np.isfinite(obs)
        new_cells = seen & (self.hits == 0)
        old_cells = seen & (self.hits > 0)

        # High-outlier gate: a single point far ABOVE the running estimate on a
        # well-seen cell is a fl*ier / moving object, not new ground -- drop it.
        if self.outlier_gate > 0.0:
            bad = old_cells & (self.hits >= self.outlier_min_hits) \
                & (obs - self.height > self.outlier_gate)
            old_cells &= ~bad

        self.height[new_cells] = obs[new_cells]
        if self.fusion == 'ema':
            a = self.ema_alpha
            self.height[old_cells] = (1.0 - a) * self.height[old_cells] \
                + a * obs[old_cells]
        else:  # 'max' -- running max: converges immediately, never lags on revisit
            self.height[old_cells] = np.maximum(self.height[old_cells],
                                                obs[old_cells])
        self.hits[seen] += 1

        # --- 2e: derived layers, only meaningful where we have data ---
        valid = self.hits >= self.min_hits
        slope_deg, step, rough = self.derive_layers(valid)

        if self.publish_debug_layers:
            # NOTE: calib_checker.py hard-codes these vis ranges to decode the grids.
            slope_unknown = ~valid | ~np.isfinite(slope_deg)
            self.debug_height_pub.publish(
                self.to_occ(self.height, -2.0, 2.0, stamp))
            self.debug_slope_pub.publish(
                self.to_occ(np.nan_to_num(slope_deg), 0.0, 45.0, stamp, slope_unknown))
            self.debug_step_pub.publish(self.to_occ(step, 0.0, 0.50, stamp, ~valid))
            self.debug_rough_pub.publish(self.to_occ(rough, 0.0, 0.15, stamp, ~valid))

        # --- 2f: combine layers -> one 0..100 cost, worst-of ---
        def ramp(v, lo, hi):
            return np.clip((v - lo) / (hi - lo), 0.0, 1.0)

        c_slope = ramp(np.nan_to_num(slope_deg), self.slope_free_deg, self.slope_lethal_deg)
        c_step = (step > self.step_lethal_m).astype(np.float32)
        c_rough = ramp(rough, self.rough_free, self.rough_lethal)
        badness = np.maximum.reduce([c_slope, c_step, c_rough])

        cost = (badness * 100.0).astype(np.int8)
        cost[~valid] = -1
        g = self._occ_msg(stamp)
        g.data = cost.ravel().tolist()
        self.cost_pub.publish(g)

    # ------------------------------------------------------------------ #
    def _smooth(self, h):
        """NaN-aware 3x3 box blur, repeated smooth_iters times."""
        for _ in range(self.smooth_iters):
            stk = np.stack([np.roll(np.roll(h, dr, 0), dc, 1)
                            for dr in (-1, 0, 1) for dc in (-1, 0, 1)])
            h = np.nanmean(stk, axis=0)
        return h

    def _slope_planefit(self, h, valid):
        """Slope (deg) from a least-squares plane fit over the 3x3 neighbourhood.

        Uses only the cells that are actually filled, so a hole next to a cell
        no longer forces its slope to 0 the way np.gradient + nan_to_num did.
        A cell needs >= slope_min_pts filled neighbours or it stays unknown.
        """
        r = self.resolution
        n = np.zeros_like(h)
        Sx = np.zeros_like(h); Sy = np.zeros_like(h)
        Sxx = np.zeros_like(h); Sxy = np.zeros_like(h); Syy = np.zeros_like(h)
        Sz = np.zeros_like(h); Sxz = np.zeros_like(h); Syz = np.zeros_like(h)
        for dr in (-1, 0, 1):
            for dc in (-1, 0, 1):
                m = np.roll(np.roll(valid, dr, 0), dc, 1).astype(np.float32)
                z = np.roll(np.roll(np.where(valid, h, 0.0), dr, 0), dc, 1) * m
                x = dc * r
                y = dr * r
                n += m
                Sx += m * x; Sy += m * y
                Sxx += m * x * x; Sxy += m * x * y; Syy += m * y * y
                Sz += z; Sxz += z * x; Syz += z * y
        # per-cell normal equations  [[Sxx Sxy Sx],[Sxy Syy Sy],[Sx Sy n]] b = [Sxz Syz Sz]
        A = np.stack([np.stack([Sxx, Sxy, Sx], -1),
                      np.stack([Sxy, Syy, Sy], -1),
                      np.stack([Sx, Sy, n], -1)], -2)
        rhs = np.stack([Sxz, Syz, Sz], -1)[..., None]
        A = A + np.eye(3, dtype=A.dtype) * 1e-6      # nudge singular (collinear) cells
        sol = (np.linalg.pinv(A) @ rhs)[..., 0]
        slope = np.degrees(np.arctan(np.hypot(sol[..., 0], sol[..., 1])))
        slope[n < self.slope_min_pts] = np.nan
        return slope

    def derive_layers(self, valid):
        """Slope (deg), max step to a neighbour (m), local roughness (m stddev).

        step and roughness run on the RAW fused height so sudden hazards (ravine
        rims, rocks) are reported at full magnitude. Only slope is optionally
        low-passed (smooth_iters), since a gradient amplifies residual jitter.
        """
        h = np.where(valid, self.height, np.nan).astype(np.float32)

        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)  # all-NaN slices early on

            hs = self._smooth(h) if self.smooth_iters > 0 else h

            if self.slope_method == 'gradient':
                dzdy, dzdx = np.gradient(hs, self.resolution)
                slope_deg = np.degrees(np.arctan(np.hypot(dzdx, dzdy)))
            else:  # 'planefit' -- hole-tolerant local least-squares plane
                slope_deg = self._slope_planefit(hs, np.isfinite(hs))

            # step: largest absolute height jump to a 4-neighbour (RAW height)
            up = np.abs(h - np.roll(h, 1, axis=0))
            down = np.abs(h - np.roll(h, -1, axis=0))
            left = np.abs(h - np.roll(h, 1, axis=1))
            right = np.abs(h - np.roll(h, -1, axis=1))
            step = np.nanmax(np.stack([up, down, left, right]), axis=0)
            step[0, :] = step[-1, :] = step[:, 0] = step[:, -1] = 0.0  # roll wraps; kill border

            # roughness: stddev of RAW height over the 8-neighbourhood
            shifts = [np.roll(np.roll(h, dr, 0), dc, 1)
                      for dr in (-1, 0, 1) for dc in (-1, 0, 1)]
            rough = np.nanstd(np.stack(shifts), axis=0)

        # slope_deg keeps its NaNs (caller builds the unknown mask from them);
        # step/rough are zeroed where unseen -- the ~valid mask hides those.
        return (slope_deg, np.nan_to_num(step), np.nan_to_num(rough))

    # ------------------------------------------------------------------ #
    def _occ_msg(self, stamp):
        """An OccupancyGrid with header + info filled, data left empty."""
        g = OccupancyGrid()
        g.header.stamp = stamp
        g.header.frame_id = self.map_frame
        g.info.resolution = self.resolution
        g.info.width = self.W
        g.info.height = self.H
        g.info.origin.position.x = self.origin_x
        g.info.origin.position.y = self.origin_y
        g.info.origin.orientation.w = 1.0
        return g

    def to_occ(self, values, vmin, vmax, stamp, unknown=None):
        """Scale a float grid to a 0-100 OccupancyGrid; -1 where `unknown` (or hits == 0)."""
        if unknown is None:
            unknown = self.hits == 0
        g = self._occ_msg(stamp)
        scaled = np.clip((values - vmin) / (vmax - vmin), 0.0, 1.0) * 100.0
        out = scaled.astype(np.int8)
        out[unknown] = -1
        g.data = out.ravel().tolist()
        return g


def main(args=None):
    rclpy.init(args=args)
    node = TraversabilityNode()
    # Multi-threaded so lookup_transform()'s timeout wait in cloud_cb doesn't
    # starve the /tf listener callback.
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
