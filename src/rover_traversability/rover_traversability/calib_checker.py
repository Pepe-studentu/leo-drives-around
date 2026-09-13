#!/usr/bin/env python3
"""Analytic checker for the traversability pipeline.

Run against gazebo_stuff/launch/calibration.launch.py . The calibration world has
known geometry, so we can state exactly what slope / step / height every cell
should read, compare the live debug grids against that, and print PASS/FAIL.

  ros2 run rover_traversability calib_checker
  ros2 run rover_traversability calib_checker --once      # one report then exit
"""
import sys
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid

# --- vis ranges the node uses in to_occ(); keep in sync with traversability_node.py
SLOPE_VMAX = 45.0      # deg   -> slope_deg = val/100 * 45
STEP_VMAX = 0.50       # m     -> step_m    = val/100 * 0.50
H_VMIN, H_VMAX = -2.0, 2.0   # m -> height_m = val/100 * 4 - 2

RAMP_DEG = 20.0
TAN20 = np.tan(np.radians(RAMP_DEG))
RAMP_TOE_X = 1.5                  # ramp meets ground here (z = 0), ascends for +x

def ramp_h(x):
    return (x - RAMP_TOE_X) * TAN20

# Regions of interest: name -> (x0, x1, y0, y1, expected_slope_deg, expected_height_m,
#                               tol_slope, tol_h). expected_height_m None -> skip height.
REGIONS = [
    # flat ground fore/aft of the rover (blind cone r~1 m; ramp toe x=1.5;
    # box045 near edge x=-2.2) -- pure ground: expect slope 0, height 0.
    # Keep >=2 cells clear of any obstacle so the 3x3 plane-fit window stays clean.
    ("ground_R",    1.05, 1.40, -1.2, 1.2,  0.0,  0.00, 3.0, 0.05),
    ("ground_L",   -1.90, -1.10, -1.2, 1.2,  0.0,  0.00, 3.0, 0.05),
    # the 20 deg ramp face (portion a -20..+3 lidar actually paints)
    ("ramp_20deg",  1.65, 2.70, -2.0, 2.0, 20.0, None, 4.0, 0.10),
]
# Box tops are too small at ~2 m to sample reliably; the *_edge step checks
# below are the real validation for the boxes.
# ramp height is x-dependent -> filled in per-cell in report()

# Step checked on a thin ring just OUTSIDE each box footprint (half-extent 0.5 m).
# name -> (cx, cy, half, expected_step_m, tol)
STEP_RINGS = [
    ("box015_edge", 0.0, -2.0, 0.50, 0.15, 0.05),
    ("box030_edge", 0.0,  2.0, 0.50, 0.30, 0.06),
    ("box045_edge", -2.7, 0.0, 0.50, 0.45, 0.08),
]


class Checker(Node):
    def __init__(self, once):
        super().__init__('calib_checker')
        self.once = once
        self.grids = {}
        q = QoSProfile(depth=1)
        q.durability = DurabilityPolicy.TRANSIENT_LOCAL
        for key, topic in (('slope', '/traversability/debug_slope'),
                           ('step',  '/traversability/debug_step'),
                           ('height', '/traversability/debug_height')):
            self.create_subscription(
                OccupancyGrid, topic, lambda m, k=key: self.grids.__setitem__(k, m), q)
        self.create_timer(2.0, self.report)

    # -- helpers ---------------------------------------------------------
    def _decode(self, key, scale, offset=0.0):
        """OccupancyGrid -> (values float array, known mask, info)."""
        g = self.grids[key]
        raw = np.array(g.data, dtype=np.int16).reshape(g.info.height, g.info.width)
        known = raw >= 0
        vals = np.where(known, raw.astype(np.float32) * scale + offset, np.nan)
        return vals, known, g.info

    def _cells_in_box(self, info, x0, x1, y0, y1):
        c0 = int(np.floor((x0 - info.origin.position.x) / info.resolution))
        c1 = int(np.ceil((x1 - info.origin.position.x) / info.resolution))
        r0 = int(np.floor((y0 - info.origin.position.y) / info.resolution))
        r1 = int(np.ceil((y1 - info.origin.position.y) / info.resolution))
        rr, cc = np.mgrid[r0:r1, c0:c1]
        return rr.ravel(), cc.ravel()

    # -- main ----------------------------------------------------------
    def report(self):
        need = {'slope', 'step', 'height'}
        if not need.issubset(self.grids):
            self.get_logger().info(f"waiting for grids: have {sorted(self.grids)}")
            return

        slope, s_known, info = self._decode('slope', SLOPE_VMAX / 100.0)
        step, _, _ = self._decode('step', STEP_VMAX / 100.0)
        height, _, _ = self._decode('height', H_VMAX * 2 / 100.0, H_VMIN)

        print("\n" + "=" * 78)
        print(f"grid {info.width}x{info.height} @ {info.resolution:.3f} m  "
              f"origin ({info.origin.position.x:.1f}, {info.origin.position.y:.1f})")
        print("-" * 78)
        print(f"{'region':<14} {'n':>4} {'cov%':>5} | "
              f"{'slope meas':>13} {'exp':>5} | {'height meas':>13} {'exp':>6} | verdict")
        print("-" * 78)

        allpass = True
        for (name, x0, x1, y0, y1, exp_sl, exp_h, tsl, th) in REGIONS:
            rr, cc = self._cells_in_box(info, x0, x1, y0, y1)
            m = s_known[rr, cc]
            n_tot, n_k = len(rr), int(m.sum())
            cov = 100.0 * n_k / max(n_tot, 1)
            if n_k < 5:
                print(f"{name:<14} {n_k:>4} {cov:>5.0f} | {'(no data)':>13} "
                      f"{exp_sl:>5.0f} | {'':>13} {'':>6} | \033[93mSKIP\033[0m")
                allpass = False
                continue
            sl = slope[rr, cc][m]
            sl_mean, sl_sd = float(np.mean(sl)), float(np.std(sl))
            ok_sl = abs(sl_mean - exp_sl) <= tsl
            # per-cell expected height: constant, or ramp_h(x) for the ramp region
            px = info.origin.position.x + (cc + 0.5) * info.resolution
            exp_h_cells = np.full(len(rr), exp_h if exp_h is not None else np.nan)
            if name == 'ramp_20deg':
                exp_h_cells = ramp_h(px)
            hstr, hexp_str, ok_h = "", "", True
            hv = height[rr, cc]
            fin = m & np.isfinite(hv) & np.isfinite(exp_h_cells)
            if fin.any():
                err = hv[fin] - exp_h_cells[fin]
                e_mean, e_sd = float(np.mean(err)), float(np.std(err))
                hstr = f"err {e_mean:+.3f}+-{e_sd:.3f}"
                hexp_str = "ramp(x)" if name == 'ramp_20deg' else f"{exp_h:+.2f}"
                ok_h = abs(e_mean) <= th
            verdict = "\033[92mPASS\033[0m" if (ok_sl and ok_h) else "\033[91mFAIL\033[0m"
            allpass &= ok_sl and ok_h
            print(f"{name:<14} {n_k:>4} {cov:>5.0f} | "
                  f"{sl_mean:>6.1f}+-{sl_sd:<4.1f} {exp_sl:>5.0f} | "
                  f"{hstr:>13} {hexp_str:>7} | {verdict}")

        print("-" * 78)
        for (name, cx, cy, half, exp_st, tst) in STEP_RINGS:
            # ring = cells whose centre is within [half, half+2*res] of the box centre
            res = info.resolution
            rr, cc = self._cells_in_box(info, cx - half - 3 * res, cx + half + 3 * res,
                                        cy - half - 3 * res, cy + half + 3 * res)
            px = info.origin.position.x + (cc + 0.5) * res
            py = info.origin.position.y + (rr + 0.5) * res
            cheb = np.maximum(np.abs(px - cx), np.abs(py - cy))
            ring = (cheb >= half - res) & (cheb <= half + 2 * res)
            rr, cc = rr[ring], cc[ring]
            m = s_known[rr, cc]
            n_k = int(m.sum())
            if n_k < 3:
                print(f"{name:<14} {n_k:>4}   -- | {'(no data)':>13} "
                      f"step exp {exp_st:.2f} | \033[93mSKIP\033[0m")
                allpass = False
                continue
            st = step[rr, cc][m]
            st_max, st_p75 = float(np.max(st)), float(np.percentile(st, 75))
            # the rim should produce a step close to the box height somewhere on the ring
            ok = abs(st_max - exp_st) <= tst
            verdict = "\033[92mPASS\033[0m" if ok else "\033[91mFAIL\033[0m"
            allpass &= ok
            print(f"{name:<14} {n_k:>4}   -- | step max {st_max:.3f} p75 {st_p75:.3f} "
                  f"| exp {exp_st:.2f} +-{tst:.2f} | {verdict}")

        print("=" * 78)
        print("OVERALL:", "\033[92mALL PASS\033[0m" if allpass else "\033[91mFAILURES ABOVE\033[0m")
        print("=" * 78)
        if self.once:
            rclpy.shutdown()


def main():
    once = '--once' in sys.argv
    rclpy.init()
    node = Checker(once)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
