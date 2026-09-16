#!/usr/bin/env python3
"""Publishes the Marsyard 2022 3D ground truth mesh marker for RViz visualization."""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from visualization_msgs.msg import Marker


class MarsyardMeshPublisher(Node):

    def __init__(self):
        super().__init__('marsyard_mesh_publisher')

        self.declare_parameter(
            'mesh_resource',
            'package://gazebo_stuff/models/marsyard2022_ground_truth_solid.obj'
        )
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('color_r', 0.76)
        self.declare_parameter('color_g', 0.50)
        self.declare_parameter('color_b', 0.32)
        self.declare_parameter('color_a', 1.0)
        self.declare_parameter('publish_rate', 0.5)

        self.mesh_resource = self.get_parameter('mesh_resource').value
        self.frame_id = self.get_parameter('frame_id').value
        self.color_r = float(self.get_parameter('color_r').value)
        self.color_g = float(self.get_parameter('color_g').value)
        self.color_b = float(self.get_parameter('color_b').value)
        self.color_a = float(self.get_parameter('color_a').value)
        publish_rate = float(self.get_parameter('publish_rate').value)

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self.publisher = self.create_publisher(
            Marker, '/marsyard/ground_truth_mesh', qos
        )

        self.timer = self.create_timer(1.0 / publish_rate, self.publish_mesh)
        self.get_logger().info(
            f'Marsyard Mesh Publisher initialized. Mesh: {self.mesh_resource} in frame {self.frame_id}'
        )

    def publish_mesh(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'marsyard_ground_truth'
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD
        marker.mesh_resource = self.mesh_resource
        marker.mesh_use_embedded_materials = False

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0

        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.color.r = self.color_r
        marker.color.g = self.color_g
        marker.color.b = self.color_b
        marker.color.a = self.color_a

        self.publisher.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = MarsyardMeshPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
