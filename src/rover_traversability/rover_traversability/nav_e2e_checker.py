#!/usr/bin/env python3
"""End-to-end Nav2 checker for the toy/calibration world.

Passive observer: doesn't send goals itself (drive with RViz's 2D Nav Goal /
Nav2 Goal tool against gazebo_stuff/launch/calibration_nav.launch.py). Watches
the global plan and the robot's pose and reports, against the calibration
world's known geometry (same box/ramp ground truth as calib_checker.py),
whether Nav2 actually avoided the hazards and reached the goal.

  ros2 run rover_traversability nav_e2e_checker
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener, TransformException

# Known hazards in calibration.sdf (see calib_checker.py for derivation).
# Boxes are 1.0x1.0 m, centered at (cx, cy); half-extent 0.5 m each.
BOXES = [
    ("box015", 0.0, -2.0, 0.5),
    ("box030", 0.0, 2.0, 0.5),
    ("box045", -2.7, 0.0, 0.5),
]

GOAL_XY_TOLERANCE = 0.4  # matches nav2_params.yaml general_goal_checker
STABLE_SECONDS = 1.0     # how long the robot must stay within tolerance


class NavE2EChecker(Node):
    def __init__(self):
        super().__init__('nav_e2e_checker')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        q = QoSProfile(depth=1)
        q.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(Path, '/plan', self.on_plan, q)

        self.goal = None            # (x, y)
        self.goal_start_time = None
        self.path_len = 0.0
        self.straight_line_dist = 0.0
        self.hazard_hit = None      # name of first box the path crossed, or None
        self.within_tol_since = None
        self.reported = False

        self.create_timer(0.2, self.check_pose)
        self.get_logger().info('nav_e2e_checker running — waiting for a plan...')

    def on_plan(self, msg: Path):
        if not msg.poses:
            return
        pts = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        start = pts[0]
        goal = pts[-1]

        self.goal = goal
        self.goal_start_time = time.time()
        self.within_tol_since = None
        self.reported = False
        self.straight_line_dist = math.hypot(goal[0] - start[0], goal[1] - start[1])
        self.path_len = sum(
            math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
            for i in range(len(pts) - 1)
        )
        self.hazard_hit = self._first_hazard_crossed(pts)

        status = f"HAZARD ({self.hazard_hit})" if self.hazard_hit else "clear"
        self.get_logger().info(
            f"new plan -> goal ({goal[0]:.2f}, {goal[1]:.2f}), "
            f"length {self.path_len:.2f}m (straight-line {self.straight_line_dist:.2f}m), "
            f"hazard check: {status}"
        )

    @staticmethod
    def _first_hazard_crossed(pts):
        for name, cx, cy, half in BOXES:
            for (x, y) in pts:
                if abs(x - cx) <= half and abs(y - cy) <= half:
                    return name
        return None

    def check_pose(self):
        if self.goal is None or self.reported:
            return
        try:
            t: TransformStamped = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
        except TransformException:
            return

        x = t.transform.translation.x
        y = t.transform.translation.y
        dist = math.hypot(x - self.goal[0], y - self.goal[1])

        now = time.time()
        if dist <= GOAL_XY_TOLERANCE:
            if self.within_tol_since is None:
                self.within_tol_since = now
            elif now - self.within_tol_since >= STABLE_SECONDS:
                self._report(success=True, duration=now - self.goal_start_time)
        else:
            self.within_tol_since = None

    def _report(self, success, duration):
        self.reported = True
        verdict = "\033[92mPASS\033[0m" if (success and not self.hazard_hit) else "\033[91mFAIL\033[0m"
        print("\n" + "=" * 60)
        print("NAV E2E CHECK")
        print("-" * 60)
        print(f"  goal reached          : {success}")
        print(f"  path crossed hazard   : {self.hazard_hit or 'none'}")
        print(f"  time to goal          : {duration:.1f} s")
        print(f"  path length           : {self.path_len:.2f} m "
              f"(straight-line {self.straight_line_dist:.2f} m, "
              f"ratio {self.path_len / max(self.straight_line_dist, 1e-3):.2f})")
        print(f"  VERDICT               : {verdict}")
        print("=" * 60 + "\n")


def main():
    rclpy.init()
    node = NavE2EChecker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
