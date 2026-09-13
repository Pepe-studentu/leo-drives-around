#!/usr/bin/env python3
import json
import math
import time
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy


# ==============================================================================
# MARSYARD 2022 NAMED WAYPOINT REGISTRY (X, Y in meters, Yaw in radians)
# ==============================================================================
NAMED_WAYPOINTS = {
    'base_station':    (-10.09,  9.33,  0.0),       # Starting Landing Site
    'inclined_mountain': (5.9531,  -12.134,  0.78),      # Inclined mountain
    'edge_mountain': (14.0, -4.16, -1.57),     
    'medium_crater':     (6.92, 11.3,  3.14), 
    'between_dunes':     (-9.27, -2.86,  1.57),
    'crater_edge':    (-4.65, -20.011, 0.0),
}

DEFAULT_MISSION_ORDER = [
    'base_station',
    'medium_crater',
    'edge_mountain',
    'inclined_mountain',
    'between_dunes',
    'base_station',
]


def calculate_perpendicular_distance(px, py, ax, ay, bx, by):
    """Calculate perpendicular distance from point P(px, py) to line segment AB."""
    dx = bx - ax
    dy = by - ay
    line_len_sq = dx * dx + dy * dy

    if line_len_sq < 1e-6:
        # Segment start and end are identical
        return math.hypot(px - ax, py - ay)

    # Cross-product formula for point-to-line distance
    num = abs(dy * px - dx * py + bx * ay - by * ax)
    denom = math.sqrt(line_len_sq)
    return num / denom


def euler_to_quaternion(yaw):
    """Convert a planar yaw angle (radians) to a ROS quaternion (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def main():
    rclpy.init()

    # Create Navigator
    navigator = BasicNavigator()

    print("\n" + "="*70)
    print("ERC MARSYARD AUTONOMOUS MISSION COORDINATOR & TEST BENCH")
    print("="*70)

    # 1. Wait for Nav2 to be Active
    print("Waiting for Nav2 active state...")
    navigator.waitUntilNav2Active(localizer='robot_localization')
    print("✅ Nav2 is fully Active and ready for autonomous mission!")

    # 2. Define Mission Sequence
    mission_sequence = DEFAULT_MISSION_ORDER
    print(f"\n📋 Planned Mission Plan ({len(mission_sequence)} Waypoints):")
    for idx, name in enumerate(mission_sequence, 1):
        x, y, yaw = NAMED_WAYPOINTS.get(name, (0.0, 0.0, 0.0))
        print(f"  [{idx}] {name.upper():<16} -> X: {x:>5.1f}m, Y: {y:>5.1f}m, Yaw: {math.degrees(yaw):>5.1f}°")

    # Metrics Accumulators
    all_cte_samples = []
    max_cte_global = 0.0
    total_distance_traveled = 0.0
    waypoints_reached = 0
    segment_reports = []

    mission_start_time = time.time()
    last_pose_x, last_pose_y = 0.0, 0.0

    # 3. Execute Mission Loop
    for seg_idx, wp_name in enumerate(mission_sequence, 1):
        if wp_name not in NAMED_WAYPOINTS:
            print(f"⚠️ Warning: Waypoint '{wp_name}' not in registry! Skipping.")
            continue

        target_x, target_y, target_yaw = NAMED_WAYPOINTS[wp_name]

        # Segment Start Point
        start_x, start_y = last_pose_x, last_pose_y
        straight_line_dist = math.hypot(target_x - start_x, target_y - start_y)

        # Build Nav2 PoseStamped Goal
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(target_x)
        goal_pose.pose.position.y = float(target_y)
        goal_pose.pose.position.z = 0.0
        qx, qy, qz, qw = euler_to_quaternion(target_yaw)
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        print("\n" + "-"*70)
        print(f"📍 Segment {seg_idx}/{len(mission_sequence)}: Navigating to [{wp_name.upper()}] (Dist: {straight_line_dist:.2f}m)...")

        # Send Navigation Goal
        navigator.goToPose(goal_pose)

        seg_cte_samples = []
        seg_start_time = time.time()
        seg_distance = 0.0
        prev_x, prev_y = start_x, start_y

        # Monitor Progress & Compute Cross-Track Error
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()

            # Query current pose if available
            if feedback:
                # Use current position to calculate distance traveled and CTE
                curr_pose = feedback.current_pose.pose.position
                cx, cy = curr_pose.x, curr_pose.y

                # Track odometry distance
                step_dist = math.hypot(cx - prev_x, cy - prev_y)
                if step_dist > 0.01:
                    seg_distance += step_dist
                    total_distance_traveled += step_dist
                    prev_x, prev_y = cx, cy

                # Calculate instantaneous Cross-Track Error (CTE)
                cte = calculate_perpendicular_distance(cx, cy, start_x, start_y, target_x, target_y)
                seg_cte_samples.append(cte)
                all_cte_samples.append(cte)
                max_cte_global = max(max_cte_global, cte)

                # ETA and remaining distance
                rem_dist = feedback.distance_remaining
                print(f"  🚗 En route to {wp_name}: Rem: {rem_dist:>4.2f}m | CTE: {cte:>4.2f}m", end='\r')

            time.sleep(0.2)

        # Check Result
        result = navigator.getResult()
        seg_duration = time.time() - seg_start_time
        last_pose_x, last_pose_y = target_x, target_y

        if result == TaskResult.SUCCEEDED:
            waypoints_reached += 1
            avg_seg_cte = (sum(seg_cte_samples) / len(seg_cte_samples)) if seg_cte_samples else 0.0
            max_seg_cte = max(seg_cte_samples) if seg_cte_samples else 0.0

            print(f"\n  🎯 Reached [{wp_name.upper()}]! (Time: {seg_duration:.1f}s, Traveled: {seg_distance:.2f}m, Mean CTE: {avg_seg_cte:.2f}m)")

            # Simulate Scientific Operation at intermediate sites
            if wp_name != 'base_station':
                print(f"  🔬 [SCIENCE OPERATION] Analyzing Martian Soil at '{wp_name}'... (3s pause)")
                time.sleep(3.0)

            segment_reports.append({
                'waypoint': wp_name,
                'status': 'SUCCEEDED',
                'duration_sec': round(seg_duration, 2),
                'planned_dist_m': round(straight_line_dist, 2),
                'traveled_dist_m': round(seg_distance, 2),
                'mean_cte_m': round(avg_seg_cte, 3),
                'max_cte_m': round(max_seg_cte, 3),
            })
        elif result == TaskResult.CANCELED:
            print(f"\n  ⚠️ Task to [{wp_name}] was CANCELED!")
            segment_reports.append({'waypoint': wp_name, 'status': 'CANCELED'})
        elif result == TaskResult.FAILED:
            print(f"\n  ❌ Failed to reach [{wp_name}]!")
            segment_reports.append({'waypoint': wp_name, 'status': 'FAILED'})

    # 4. Final Mission Metrics Computation
    total_mission_time = time.time() - mission_start_time
    success_rate = (waypoints_reached / len(mission_sequence)) * 100.0
    mean_cte = (sum(all_cte_samples) / len(all_cte_samples)) if all_cte_samples else 0.0
    rmse_cte = math.sqrt(sum(c*c for c in all_cte_samples) / len(all_cte_samples)) if all_cte_samples else 0.0

    # Print Final Summary Table
    print("\n" + "="*70)
    print("📊 FINAL MARS MISSION BENCHMARK REPORT")
    print("="*70)
    print(f"  • Mission Success Rate      : {waypoints_reached}/{len(mission_sequence)} ({success_rate:.1f}%)")
    print(f"  • Total Mission Duration    : {total_mission_time:.1f} seconds")
    print(f"  • Total Distance Traveled   : {total_distance_traveled:.2f} meters")
    print(f"  • Mean Cross-Track Error    : {mean_cte:.3f} meters")
    print(f"  • Max Cross-Track Error     : {max_cte_global:.3f} meters (Observed during rock avoidance)")
    print(f"  • RMSE Cross-Track Error    : {rmse_cte:.3f} meters")
    print("="*70)

    # Save JSON Report
    report_data = {
        'mission_success_rate_percent': round(success_rate, 2),
        'waypoints_reached': waypoints_reached,
        'waypoints_total': len(mission_sequence),
        'total_duration_sec': round(total_mission_time, 2),
        'total_distance_m': round(total_distance_traveled, 2),
        'mean_cross_track_error_m': round(mean_cte, 4),
        'max_cross_track_error_m': round(max_cte_global, 4),
        'rmse_cross_track_error_m': round(rmse_cte, 4),
        'segments': segment_reports
    }

    report_path = '/home/ws/mission_report.json'
    with open(report_path, 'w') as f:
        json.dump(report_data, f, indent=2)
    print(f"💾 Full test results saved to: {report_path}\n")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
