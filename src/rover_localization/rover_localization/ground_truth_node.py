#!/usr/bin/env python3
"""Ground truth bridge node.

Translates Gazebo's SceneBroadcaster dynamic pose topic into standard ROS 2
ground-truth messages:
1. /ground_truth/pose (geometry_msgs/msg/PoseStamped in 'map' frame)
2. TF transform: map -> ground_truth_base
"""
import math
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster


class GroundTruthNode(Node):
    def __init__(self):
        super().__init__('ground_truth_node')

        self.declare_parameter('target_model', 'leo_rover')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('ground_truth_frame', 'ground_truth_base')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('publish_base_tf', False)

        self.target_model = self.get_parameter('target_model').get_parameter_value().string_value
        self.map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self.gt_frame = self.get_parameter('ground_truth_frame').get_parameter_value().string_value
        self.publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value
        self.publish_base_tf = self.get_parameter('publish_base_tf').get_parameter_value().bool_value

        self.pose_pub = self.create_publisher(PoseStamped, '/ground_truth/pose', 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None

        # Gazebo bridge publishes dynamic pose as TFMessage on /gazebo/dynamic_pose
        self.create_subscription(
            TFMessage,
            '/gazebo/dynamic_pose',
            self.dynamic_pose_cb,
            10
        )
        self.last_pub_time = 0.0
        self.get_logger().info(f"Ground truth node initialized. Target model: {self.target_model}")

    def dynamic_pose_cb(self, msg: TFMessage):
        if not msg.transforms:
            return

        now_sec = self.get_clock().now().nanoseconds * 1e-9
        if now_sec - self.last_pub_time < 0.05:  # Throttle from 58 Hz to 20 Hz
            return
        self.last_pub_time = now_sec

        # In Gazebo SceneBroadcaster dynamic_pose/info, the model pose (leo_rover)
        # is the first transform in the vector (index 0).
        # We also check child_frame_id if populated.
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

        now = self.get_clock().now().to_msg()

        # 1. Publish PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.stamp = now
        pose_msg.header.frame_id = self.map_frame
        pose_msg.pose.position.x = target_tf.transform.translation.x
        pose_msg.pose.position.y = target_tf.transform.translation.y
        pose_msg.pose.position.z = target_tf.transform.translation.z
        pose_msg.pose.orientation = target_tf.transform.rotation
        self.pose_pub.publish(pose_msg)

        # 2. Broadcast TF: map -> ground_truth_base
        if self.publish_tf:
            tf_stamped = TransformStamped()
            tf_stamped.header.stamp = now
            tf_stamped.header.frame_id = self.map_frame
            tf_stamped.child_frame_id = self.gt_frame
            tf_stamped.transform = target_tf.transform
            self.tf_broadcaster.sendTransform(tf_stamped)

        # 3. Broadcast TF: odom -> base_footprint (for ground truth testing)
        if self.publish_base_tf:
            base_tf = TransformStamped()
            base_tf.header.stamp = now
            base_tf.header.frame_id = 'odom'
            base_tf.child_frame_id = 'base_footprint'
            base_tf.transform = target_tf.transform
            self.tf_broadcaster.sendTransform(base_tf)


def main(args=None):
    rclpy.init(args=args)
    node = GroundTruthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
