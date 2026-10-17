"""Bayesian height map + slope costmap. Pure numpy/OpenCV, no ROS imports.

All terrain reasoning lives here; traversability_node.py only moves data in and
out of ROS.

Height map
----------
Each cell (r, c) of a 2.5D grid in the map frame holds an estimated height and
a variance. Every scan is reduced to one mean height per cell, and fused with
the stored estimate by a scalar Kalman update. Variance of one observation:

    sigma_obs^2 = sigma_sensor^2 + (range * sigma_angular)^2 + k_drift * dt_oracle

dt_oracle is the time since the last global (satellite) fix, so scans taken
right after a fix are trusted more than scans taken long after one. Close-range
points have small variance and dominate distant, noisy ones. An observation
further than `mahalanobis_gate` sigmas from the estimate (and more than 0.20 m
away) is rejected; each rejection inflates the cell's variance so a real change
can eventually be absorbed.

Aging: between scans every stored variance grows by k_age * dt (capped at
max_variance), as in the predict step of a Kalman filter. A cell that has not
been re-observed for a while was written under older pose estimates, so it is
trusted less; the first scan after a global fix then overwrites it with a high
gain, while a cell seen every scan keeps a small variance and barely moves.

Costmap
-------
Slope only. Gaps smaller than the rover are closed morphologically, heights are
box-smoothed over 3x3, and slope comes from central differences.

    slope <= slope_free_deg                       : cost 0
    slope_free_deg  < slope <= slope_warn_deg     : cost slope_warn_cost
    slope_warn_deg  < slope <= slope_lethal_deg   : ramps slope_warn_cost -> 100
    slope > slope_lethal_deg                      : cost 100 (lethal)

Unexplored cells cost 0 (the planner is allowed to explore them), and cells
under the rover itself are forced to 0. A step-height layer is computed for
debugging only; it does NOT enter the cost.
"""
from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class HeightMapConfig:
    resolution: float = 0.10
    map_size_x: float = 50.0
    map_size_y: float = 50.0
    origin_x: float = -25.0
    origin_y: float = -25.0

    sigma_sensor: float = 0.02      # base lidar depth noise std dev (m)
    sigma_angular: float = 0.01     # lever-arm uncertainty per metre of range (rad)
    k_drift: float = 0.005          # variance growth (m^2/s) since the last global fix
    min_variance: float = 0.0001    # variance floor (m^2), keeps the filter responsive
    mahalanobis_gate: float = 3.5   # outlier rejection threshold (std devs)
    k_age: float = 0.002            # stored-variance growth (m^2/s): old cells become easier to overwrite
    max_variance: float = 0.05      # cap on aging (m^2), so a never-revisited cell stays finite

    slope_free_deg: float = 30.0
    slope_warn_deg: float = 35.0
    slope_lethal_deg: float = 45.0
    slope_warn_cost: float = 10.0
    rover_footprint_radius: float = 0.45   # cells this close to the rover are forced free

    spawn_bootstrap_radius: float = 1.5    # known-ground disc under the rover at start (m)
    lidar_height: float = 0.23             # lidar height above the ground at spawn (m)


class HeightMap:

    def __init__(self, cfg: HeightMapConfig):
        self.cfg = cfg
        self.W = int(round(cfg.map_size_x / cfg.resolution))
        self.H = int(round(cfg.map_size_y / cfg.resolution))
        self.height = np.full((self.H, self.W), -np.inf, dtype=np.float32)
        self.var = np.full((self.H, self.W), np.inf, dtype=np.float32)
        self.hits = np.zeros((self.H, self.W), dtype=np.int32)
        # cell-centre coordinates, for distance masks
        self.grid_x = cfg.origin_x + (np.arange(self.W, dtype=np.float32) + 0.5) * cfg.resolution
        self.grid_y = cfg.origin_y + (np.arange(self.H, dtype=np.float32) + 0.5) * cfg.resolution

    # ------------------------------------------------------------------ #
    @property
    def filled(self):
        """Boolean mask of cells that have a height estimate."""
        return np.isfinite(self.height) & (self.var < np.inf) & (self.hits > 0)

    def _disc(self, x, y, radius):
        return ((self.grid_x[None, :] - x) ** 2 + (self.grid_y[:, None] - y) ** 2) <= radius ** 2

    def bootstrap(self, sensor_x, sensor_y, sensor_z):
        """Declare the ground under the rover known, so it starts on explored land.

        The lidar cannot see the ground right below itself, so without this the
        rover would start in a hole of unknown cells. Returns the ground height.
        """
        ground_z = float(sensor_z - self.cfg.lidar_height)
        mask = self._disc(sensor_x, sensor_y, self.cfg.spawn_bootstrap_radius)
        self.height[mask] = ground_z
        self.var[mask] = 0.01
        self.hits[mask] = 10
        return ground_z

    # ------------------------------------------------------------------ #
    def age(self, dt):
        """Grow the variance of every known cell by k_age * dt (call once per scan, before integrate)."""
        known = self.var < np.inf
        v = self.var[known]
        self.var[known] = np.minimum(v + self.cfg.k_age * dt, np.maximum(v, self.cfg.max_variance))

    def integrate(self, pts_map, ranges, dt_oracle):
        """Fuse one scan (N x 3 points already in the map frame) into the grid."""
        c = self.cfg
        pt_var = (c.sigma_sensor ** 2) + (ranges * c.sigma_angular) ** 2 + c.k_drift * dt_oracle

        cols = np.floor((pts_map[:, 0] - c.origin_x) / c.resolution).astype(np.int32)
        rows = np.floor((pts_map[:, 1] - c.origin_y) / c.resolution).astype(np.int32)
        ok = (cols >= 0) & (cols < self.W) & (rows >= 0) & (rows < self.H)
        if not np.any(ok):
            return
        cols, rows = cols[ok], rows[ok]
        zs = pts_map[ok, 2].astype(np.float32)
        pt_var = pt_var[ok].astype(np.float32)

        # Per-cell aggregation of this scan
        flat = rows * self.W + cols
        n_cells = self.H * self.W
        counts = np.bincount(flat, minlength=n_cells).astype(np.float32)
        seen = counts > 0
        z_sum = np.bincount(flat, weights=zs, minlength=n_cells).astype(np.float32)
        v_sum = np.bincount(flat, weights=pt_var, minlength=n_cells).astype(np.float32)
        z_obs = np.zeros(n_cells, dtype=np.float32)
        z_obs[seen] = z_sum[seen] / counts[seen]
        v_obs = np.full(n_cells, np.inf, dtype=np.float32)
        v_obs[seen] = (v_sum[seen] / counts[seen]) / np.sqrt(counts[seen])   # more points -> less variance

        seen = seen.reshape(self.H, self.W)
        z_obs = z_obs.reshape(self.H, self.W)
        v_obs = v_obs.reshape(self.H, self.W)

        # Unvisited cells: initialise directly from the observation
        new = seen & (self.var == np.inf)
        self.height[new] = z_obs[new]
        self.var[new] = v_obs[new]

        # Visited cells: Kalman fusion with Mahalanobis gating. NOTE: `new` cells are
        # included on purpose (as in the original node): they get fused with their own
        # first observation once, which halves their initial variance.
        old = seen & (self.var < np.inf)
        if np.any(old):
            h_prior, v_prior = self.height[old], self.var[old]
            h_meas, v_meas = z_obs[old], v_obs[old]

            diff = np.abs(h_meas - h_prior)
            inlier = (diff <= c.mahalanobis_gate * np.sqrt(v_prior + v_meas)) | (diff < 0.20)

            K = np.zeros_like(h_prior)
            K[inlier] = v_prior[inlier] / (v_prior[inlier] + v_meas[inlier])

            h_new = h_prior + K * (h_meas - h_prior)
            v_new = np.maximum(c.min_variance, (1.0 - K) * v_prior)
            # Rejected observation: grow uncertainty so a real change can eventually win
            v_new[~inlier] = np.minimum(1.0, v_prior[~inlier] + 0.001)

            self.height[old] = h_new
            self.var[old] = v_new

        self.hits[seen] += 1

    # ------------------------------------------------------------------ #
    def cloud_points(self):
        """(N, 4) float32 [x, y, z, intensity=z] of every filled cell, for visualisation."""
        rr, cc = np.nonzero(self.filled)
        if rr.size == 0:
            return np.zeros((0, 4), dtype=np.float32)
        cx = self.cfg.origin_x + (cc.astype(np.float32) + 0.5) * self.cfg.resolution
        cy = self.cfg.origin_y + (rr.astype(np.float32) + 0.5) * self.cfg.resolution
        cz = self.height[rr, cc]
        return np.stack([cx, cy, cz, cz], axis=1).astype(np.float32)

    # ------------------------------------------------------------------ #
    def compute_costmaps(self, rover_xy=None):
        """Return (cost, slope_layer, step_layer) as int8 (H, W) grids.

        cost is 0..100. The two layers are debug views (-1 unknown, 1..98 scaled).
        """
        c = self.cfg
        H, W, res = self.H, self.W, c.resolution
        valid = self.filled

        if not np.any(valid):
            return (np.zeros((H, W), dtype=np.int8),
                    np.full((H, W), -1, dtype=np.int8),
                    np.full((H, W), -1, dtype=np.int8))

        # 1. Close gaps smaller than the rover (3x3 = 30 cm, chassis is 45 cm)
        k3 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        valid_closed = cv2.morphologyEx(valid.astype(np.uint8), cv2.MORPH_CLOSE, k3).astype(bool)
        k_box = np.ones((3, 3), dtype=np.float32)

        # Fill the closed gaps with the mean of their valid neighbours
        gap = valid_closed & ~valid
        h_dense = np.where(valid, self.height, 0.0).astype(np.float32)
        if np.any(gap):
            h_sum = cv2.filter2D(h_dense, -1, k_box)
            m_sum = cv2.filter2D(valid.astype(np.float32), -1, k_box)
            fill = gap & (m_sum > 0)
            h_dense[fill] = h_sum[fill] / m_sum[fill]

        # 2. Smooth over the footprint to remove lidar jitter
        h_sum = cv2.filter2D(h_dense, -1, k_box)
        m_sum = cv2.filter2D(valid_closed.astype(np.float32), -1, k_box)
        h_smooth = np.zeros_like(h_dense)
        ok = valid_closed & (m_sum > 0)
        h_smooth[ok] = h_sum[ok] / m_sum[ok]

        # Central differences (one-sided at the edge of the known area)
        h_c, v_c = h_smooth[1:-1, 1:-1], valid_closed[1:-1, 1:-1]
        h_r, v_r = h_smooth[1:-1, 2:], valid_closed[1:-1, 2:]
        h_l, v_l = h_smooth[1:-1, :-2], valid_closed[1:-1, :-2]
        h_d, v_d = h_smooth[2:, 1:-1], valid_closed[2:, 1:-1]
        h_u, v_u = h_smooth[:-2, 1:-1], valid_closed[:-2, 1:-1]

        dx = np.zeros_like(h_c)
        both = v_r & v_l
        dx[both] = (h_r[both] - h_l[both]) / (2.0 * res)
        only = v_r & ~v_l & v_c
        dx[only] = (h_r[only] - h_c[only]) / res
        only = v_l & ~v_r & v_c
        dx[only] = (h_c[only] - h_l[only]) / res

        dy = np.zeros_like(h_c)
        both = v_d & v_u
        dy[both] = (h_d[both] - h_u[both]) / (2.0 * res)
        only = v_d & ~v_u & v_c
        dy[only] = (h_d[only] - h_c[only]) / res
        only = v_u & ~v_d & v_c
        dy[only] = (h_c[only] - h_u[only]) / res

        slope_deg = np.zeros((H, W), dtype=np.float32)
        slope_deg[1:-1, 1:-1] = np.where(v_c, np.degrees(np.arctan(np.hypot(dx, dy))), 0.0)

        # 3. Slope -> cost
        cost = np.zeros((H, W), dtype=np.float32)
        warn = valid_closed & (slope_deg > c.slope_free_deg) & (slope_deg <= c.slope_warn_deg)
        cost[warn] = c.slope_warn_cost
        ramp = valid_closed & (slope_deg > c.slope_warn_deg) & (slope_deg <= c.slope_lethal_deg)
        progress = (slope_deg[ramp] - c.slope_warn_deg) / (c.slope_lethal_deg - c.slope_warn_deg)
        cost[ramp] = c.slope_warn_cost + (100.0 - c.slope_warn_cost) * progress
        cost[valid_closed & (slope_deg > c.slope_lethal_deg)] = 100.0

        cost[~valid_closed] = 0.0                      # unexplored = free (planner may explore it)
        if rover_xy is not None:
            cost[self._disc(rover_xy[0], rover_xy[1], c.rover_footprint_radius)] = 0.0
        cost_grid = np.clip(cost, 0.0, 100.0).astype(np.int8)

        # 4. Debug layers (1..98 blue->red)
        slope_layer = np.full((H, W), -1, dtype=np.int8)
        s = np.clip(slope_deg / 45.0, 0.0, 1.0)
        slope_layer[valid_closed] = np.clip(1 + np.round(s[valid_closed] * 97.0), 1, 98).astype(np.int8)

        step_in = np.maximum.reduce([
            np.where(v_r & v_c, np.abs(h_r - h_c), 0.0),
            np.where(v_l & v_c, np.abs(h_l - h_c), 0.0),
            np.where(v_d & v_c, np.abs(h_d - h_c), 0.0),
            np.where(v_u & v_c, np.abs(h_u - h_c), 0.0)])
        step_m = np.zeros((H, W), dtype=np.float32)
        step_m[1:-1, 1:-1] = np.where(v_c, step_in, 0.0)
        st = np.clip(step_m / 0.30, 0.0, 1.0)
        step_layer = np.full((H, W), -1, dtype=np.int8)
        step_layer[valid_closed] = np.clip(1 + np.round(st[valid_closed] * 97.0), 1, 98).astype(np.int8)

        return cost_grid, slope_layer, step_layer
