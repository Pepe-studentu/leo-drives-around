#!/usr/bin/env python3
import json
import math
import time
from geometry_msgs.msg import Point, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from visualization_msgs.msg import Marker, MarkerArray


# ==============================================================================
# MARSYARD 2022 NAMED WAYPOINT REGISTRY (X, Y in meters, Yaw in radians)
# ==============================================================================
NAMED_WAYPOINTS = {
    'base_station':      (-10.09,   9.33,  0.0),       # Starting Landing Site
    'medium_crater':     (  6.92,  11.30,  3.14),     # Medium Crater Rim
    'edge_mountain':     ( 14.00,  -4.16, -1.57),     # Edge of Mountain
    'between_dunes':     ( -9.27,  -2.86,  1.57),     # Flat corridor between dunes
    'crater_edge':       ( -4.65, -20.01,  0.0),      # South crater edge
}

DEFAULT_MISSION_ORDER = [
    'medium_crater',
    'edge_mountain',
    'between_dunes',
    'base_station',
]


def calculate_perpendicular_distance(px, py, ax, ay, bx, by):
    """Calculate perpendicular distance from point P(px, py) to line segment AB."""
    dx = bx - ax
    dy = by - ay
    line_len_sq = dx * dx + dy * dy

    if line_len_sq < 1e-6:
        return math.hypot(px - ax, py - ay)

    num = abs(dy * px - dx * py + bx * ay - by * ax)
    denom = math.sqrt(line_len_sq)
    return num / denom


def euler_to_quaternion(yaw):
    """Convert a planar yaw angle (radians) to a ROS quaternion (x, y, z, w)."""
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def create_waypoint_markers(mission_sequence, named_waypoints, stamp):
    """Generate RViz MarkerArray showing numbered waypoints, orientations, and route."""
    marker_array = MarkerArray()

    # Route Line Strip
    line_marker = Marker()
    line_marker.header.frame_id = 'map'
    line_marker.header.stamp = stamp
    line_marker.ns = 'mission_route'
    line_marker.id = 0
    line_marker.type = Marker.LINE_STRIP
    line_marker.action = Marker.ADD
    line_marker.scale.x = 0.08  # line width
    line_marker.color.r = 1.0
    line_marker.color.g = 0.75
    line_marker.color.b = 0.1
    line_marker.color.a = 0.7

    for idx, name in enumerate(mission_sequence, 1):
        if name not in named_waypoints:
            continue
        x, y, yaw = named_waypoints[name]

        # Add point to route line
        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = 0.1
        line_marker.points.append(p)

        # 1. Ground Disc Marker
        disc = Marker()
        disc.header.frame_id = 'map'
        disc.header.stamp = stamp
        disc.ns = 'waypoint_discs'
        disc.id = idx * 10
        disc.type = Marker.CYLINDER
        disc.action = Marker.ADD
        disc.pose.position.x = float(x)
        disc.pose.position.y = float(y)
        disc.pose.position.z = 0.05
        disc.scale.x = 0.8
        disc.scale.y = 0.8
        disc.scale.z = 0.06
        disc.color.r = 1.0
        disc.color.g = 0.65
        disc.color.b = 0.0
        disc.color.a = 0.85
        marker_array.markers.append(disc)

        # 2. Orientation Arrow Marker
        arrow = Marker()
        arrow.header.frame_id = 'map'
        arrow.header.stamp = stamp
        arrow.ns = 'waypoint_headings'
        arrow.id = idx * 10 + 1
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose.position.x = float(x)
        arrow.pose.position.y = float(y)
        arrow.pose.position.z = 0.12
        qx, qy, qz, qw = euler_to_quaternion(yaw)
        arrow.pose.orientation.x = qx
        arrow.pose.orientation.y = qy
        arrow.pose.orientation.z = qz
        arrow.pose.orientation.w = qw
        arrow.scale.x = 0.6
        arrow.scale.y = 0.1
        arrow.scale.z = 0.1
        arrow.color.r = 1.0
        arrow.color.g = 0.9
        arrow.color.b = 0.2
        arrow.color.a = 0.9
        marker_array.markers.append(arrow)

        # 3. Numbered Text Label Floating Above
        text = Marker()
        text.header.frame_id = 'map'
        text.header.stamp = stamp
        text.ns = 'waypoint_labels'
        text.id = idx * 10 + 2
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(x)
        text.pose.position.y = float(y)
        text.pose.position.z = 0.65
        text.scale.z = 0.45  # Text height
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.color.a = 1.0
        text.text = f"[{idx}] {name.upper()}"
        marker_array.markers.append(text)

    marker_array.markers.append(line_marker)
    return marker_array


def main():
    rclpy.init()

    # Create Navigator
    navigator = BasicNavigator()

    print("\n" + "="*75)
    print("🚀 ERC MARSYARD AUTONOMOUS WAYPOINT MISSION RUNNER")
    print("="*75)

    # Waypoints Publisher (Transient Local so RViz gets it immediately)
    marker_qos = QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL
    )
    marker_pub = navigator.create_publisher(MarkerArray, '/mission/waypoints', marker_qos)

    # 1. Publish Waypoints to RViz
    mission_sequence = DEFAULT_MISSION_ORDER
    wp_markers = create_waypoint_markers(
        mission_sequence, NAMED_WAYPOINTS, navigator.get_clock().now().to_msg())
    marker_pub.publish(wp_markers)
    print("📍 Waypoints & mission route published to RViz on /mission/waypoints")

    # 2. Wait for Nav2 to be Active
    print("⏳ Waiting for Nav2 active state...")
    navigator.waitUntilNav2Active(localizer='robot_localization')
    print("✅ Nav2 is fully Active and ready for autonomous mission!")

    # Re-publish markers once Nav2 is up to guarantee RViz subscription catch
    marker_pub.publish(wp_markers)

    print(f"\n📋 Planned Mission Sequence ({len(mission_sequence)} Waypoints):")
    for idx, name in enumerate(mission_sequence, 1):
        x, y, yaw = NAMED_WAYPOINTS.get(name, (0.0, 0.0, 0.0))
        print(f"  [{idx}] {name.upper():<18} -> X: {x:>6.2f}m, Y: {y:>6.2f}m, Yaw: {math.degrees(yaw):>5.1f}°")

    # Metrics Accumulators
    all_cte_samples = []
    max_cte_global = 0.0
    total_distance_traveled = 0.0
    total_mission_recoveries = 0
    waypoints_reached = 0
    segment_reports = []

    mission_start_time = time.time()
    last_pose_x, last_pose_y = NAMED_WAYPOINTS['base_station'][0], NAMED_WAYPOINTS['base_station'][1]

    # 3. Execute Mission Loop
    for seg_idx, wp_name in enumerate(mission_sequence, 1):
        if wp_name not in NAMED_WAYPOINTS:
            print(f"⚠️ Warning: Waypoint '{wp_name}' not in registry! Skipping.")
            continue

        target_x, target_y, target_yaw = NAMED_WAYPOINTS[wp_name]

        # Segment Start Coordinates (from previous waypoint or spawn)
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

        prev_label = f"[{mission_sequence[seg_idx-2].upper()}]" if seg_idx > 1 else "[BASE_STATION (SPAWN)]"
        print("\n" + "-"*75)
        print(f"📍 Segment {seg_idx}/{len(mission_sequence)}: From {prev_label} -> [{wp_name.upper()}] (Planned: {straight_line_dist:.2f}m)")

        # Send Navigation Goal
        navigator.goToPose(goal_pose)

        seg_cte_samples = []
        seg_start_time = time.time()
        seg_distance = 0.0
        prev_x, prev_y = start_x, start_y
        seg_recoveries = 0
        seg_start_recoveries = 0
        first_feedback = True

        # Monitor Progress & Compute Metrics
        while not navigator.isTaskComplete():
            feedback = navigator.getFeedback()

            if feedback:
                if first_feedback:
                    seg_start_recoveries = feedback.number_of_recoveries
                    first_feedback = False

                seg_recoveries = max(0, feedback.number_of_recoveries - seg_start_recoveries)

                # Current position
                curr_pose = feedback.current_pose.pose.position
                cx, cy = curr_pose.x, curr_pose.y

                # Track distance traveled
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

                # Time and distance
                elapsed_seg = time.time() - seg_start_time
                rem_dist = feedback.distance_remaining
                print(f"  🚗 [{wp_name}]: Elapsed: {elapsed_seg:>5.1f}s | Rem: {rem_dist:>4.2f}m | CTE: {cte:>4.2f}m | Recoveries: {seg_recoveries}", end='\r')

            time.sleep(0.2)

        # Check Result
        result = navigator.getResult()
        seg_duration = time.time() - seg_start_time
        if result == TaskResult.SUCCEEDED:
            last_pose_x, last_pose_y = target_x, target_y
        else:
            last_pose_x, last_pose_y = prev_x, prev_y
        total_mission_recoveries += seg_recoveries

        avg_seg_cte = (sum(seg_cte_samples) / len(seg_cte_samples)) if seg_cte_samples else 0.0
        max_seg_cte = max(seg_cte_samples) if seg_cte_samples else 0.0

        if result == TaskResult.SUCCEEDED:
            waypoints_reached += 1
            print(f"\n  🎯 REACHED [{wp_name.upper()}]!")
            print(f"     ⏱️  Time from previous: {seg_duration:.1f} s")
            print(f"     🔄 Recoveries count:    {seg_recoveries}")
            print(f"     📏 Distance traveled:   {seg_distance:.2f} m (planned: {straight_line_dist:.2f} m)")
            print(f"     📐 Mean Cross-Track:    {avg_seg_cte:.2f} m (max: {max_seg_cte:.2f} m)")

            # Simulate Scientific Operation at intermediate exploration sites
            if wp_name != 'base_station':
                print(f"  🔬 [SCIENCE OPERATION] Inspecting target site at '{wp_name}'... (2.0s)")
                time.sleep(2.0)

            segment_reports.append({
                'segment': seg_idx,
                'waypoint': wp_name,
                'status': 'SUCCEEDED',
                'duration_sec': round(seg_duration, 2),
                'recoveries': seg_recoveries,
                'planned_dist_m': round(straight_line_dist, 2),
                'traveled_dist_m': round(seg_distance, 2),
                'mean_cte_m': round(avg_seg_cte, 3),
                'max_cte_m': round(max_seg_cte, 3),
            })
        elif result == TaskResult.CANCELED:
            print(f"\n  ⚠️ Task to [{wp_name}] was CANCELED! (Time: {seg_duration:.1f}s, Recoveries: {seg_recoveries})")
            segment_reports.append({
                'segment': seg_idx,
                'waypoint': wp_name,
                'status': 'CANCELED',
                'duration_sec': round(seg_duration, 2),
                'recoveries': seg_recoveries,
            })
        elif result == TaskResult.FAILED:
            print(f"\n  ❌ Failed to reach [{wp_name}]! (Time: {seg_duration:.1f}s, Recoveries: {seg_recoveries})")
            segment_reports.append({
                'segment': seg_idx,
                'waypoint': wp_name,
                'status': 'FAILED',
                'duration_sec': round(seg_duration, 2),
                'recoveries': seg_recoveries,
            })

    # 4. Final Mission Metrics Computation
    total_mission_time = time.time() - mission_start_time
    success_rate = (waypoints_reached / len(mission_sequence)) * 100.0
    mean_cte = (sum(all_cte_samples) / len(all_cte_samples)) if all_cte_samples else 0.0
    rmse_cte = math.sqrt(sum(c*c for c in all_cte_samples) / len(all_cte_samples)) if all_cte_samples else 0.0

    # Print Detailed Benchmark Table
    print("\n" + "="*75)
    print("📊 FINAL MARS MISSION BENCHMARK REPORT")
    print("="*75)
    print(f"{'#':<3} {'Waypoint':<18} {'Status':<10} {'Time (s)':<10} {'Recov':<7} {'Dist (m)':<10} {'Mean CTE':<10}")
    print("-" * 75)
    for rep in segment_reports:
        dur = f"{rep.get('duration_sec', 0.0):.1f}s"
        rec = str(rep.get('recoveries', 0))
        dist = f"{rep.get('traveled_dist_m', 0.0):.1f}m"
        m_cte = f"{rep.get('mean_cte_m', 0.0):.2f}m"
        print(f"{rep['segment']:<3} {rep['waypoint']:<18} {rep['status']:<10} {dur:<10} {rec:<7} {dist:<10} {m_cte:<10}")

    print("="*75)
    print(f"  • Mission Success Rate      : {waypoints_reached}/{len(mission_sequence)} ({success_rate:.1f}%)")
    print(f"  • Total Mission Duration    : {total_mission_time:.1f} seconds")
    print(f"  • Total Recoveries Executed : {total_mission_recoveries}")
    print(f"  • Total Distance Traveled   : {total_distance_traveled:.2f} meters")
    print(f"  • Mean Cross-Track Error    : {mean_cte:.3f} meters")
    print(f"  • Max Cross-Track Error     : {max_cte_global:.3f} meters")
    print(f"  • RMSE Cross-Track Error    : {rmse_cte:.3f} meters")
    print("="*75)

    # Save JSON Report
    report_data = {
        'mission_success_rate_percent': round(success_rate, 2),
        'waypoints_reached': waypoints_reached,
        'waypoints_total': len(mission_sequence),
        'total_duration_sec': round(total_mission_time, 2),
        'total_recoveries': total_mission_recoveries,
        'total_distance_m': round(total_distance_traveled, 2),
        'mean_cross_track_error_m': round(mean_cte, 4),
        'max_cross_track_error_m': round(max_cte_global, 4),
        'rmse_cross_track_error_m': round(rmse_cte, 4),
        'segments': segment_reports
    }

    report_path = '/home/ws/mission_report.json'
    try:
        with open(report_path, 'w') as f:
            json.dump(report_data, f, indent=2)
        print(f"💾 Full test results saved to: {report_path}\n")
    except Exception as e:
        print(f"Note: could not write to {report_path}: {e}")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
