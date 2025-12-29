#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

class TableMeshPublisher(Node):
    def __init__(self):
        super().__init__('table_mesh_publisher')

        # Parameters (can be overridden by launch/ros2 param)
        self.declare_parameter('mesh_resource', 'package://nantrobot_robot_sim/mesh/match_table.dae')
        self.declare_parameter('frame_id', 'world')
        self.declare_parameter('position_x', 1.5)
        self.declare_parameter('position_y', 1.0)
        self.declare_parameter('position_z', 0.0)
        self.declare_parameter('orientation_x', 0.5)
        self.declare_parameter('orientation_y', 0.5)
        self.declare_parameter('orientation_z', 0.5)
        self.declare_parameter('orientation_w', 0.5)
        self.declare_parameter('scale_x', 0.1)
        self.declare_parameter('scale_y', 0.1)
        self.declare_parameter('scale_z', 0.1)
        self.declare_parameter('pub_period', 5.0)

        marker_qos = QoSProfile(depth=10, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.table_publisher = self.create_publisher(Marker, 'table_mesh', marker_qos)

        self.timer = self.create_timer(self.get_parameter('pub_period').get_parameter_value().double_value,
                                       self.publish_table_mesh)

        # Publish immediately so transient local subscribers can receive it quickly
        self.publish_table_mesh()
        self.get_logger().info(f'Table mesh publisher started (resource={self.get_parameter("mesh_resource").get_parameter_value().string_value})')

    def publish_table_mesh(self):
        marker = Marker()
        marker.header.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "table"
        marker.id = 0
        marker.type = Marker.MESH_RESOURCE
        marker.action = Marker.ADD

        # Position & orientation
        marker.pose.position.x = float(self.get_parameter('position_x').get_parameter_value().double_value)
        marker.pose.position.y = float(self.get_parameter('position_y').get_parameter_value().double_value)
        marker.pose.position.z = float(self.get_parameter('position_z').get_parameter_value().double_value)
        marker.pose.orientation.x = float(self.get_parameter('orientation_x').get_parameter_value().double_value)
        marker.pose.orientation.y = float(self.get_parameter('orientation_y').get_parameter_value().double_value)
        marker.pose.orientation.z = float(self.get_parameter('orientation_z').get_parameter_value().double_value)
        marker.pose.orientation.w = float(self.get_parameter('orientation_w').get_parameter_value().double_value)

        # Scale
        marker.scale.x = float(self.get_parameter('scale_x').get_parameter_value().double_value)
        marker.scale.y = float(self.get_parameter('scale_y').get_parameter_value().double_value)
        marker.scale.z = float(self.get_parameter('scale_z').get_parameter_value().double_value)

        marker.mesh_resource = self.get_parameter('mesh_resource').get_parameter_value().string_value
        marker.mesh_use_embedded_materials = True

        # Persistent marker
        marker.lifetime.sec = 0

        self.table_publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = TableMeshPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()