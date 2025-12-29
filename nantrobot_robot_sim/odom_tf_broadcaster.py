#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class OdomTFBroadcaster(Node):

    def __init__(self):
        super().__init__('odom_tf_broadcaster')

        # Get the namespace to prefix frame IDs
        self.namespace = self.get_namespace()
        # Remove leading/trailing slashes and use as prefix
        self.frame_prefix = self.namespace.strip('/') + '/' if self.namespace != '/' else ''

        self.tf_broadcaster = TransformBroadcaster(self)
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(f'TF frames will use prefix: "{self.frame_prefix}"')

    def odom_callback(self, msg: Odometry):
        t = TransformStamped()
        # use of the pc/raspy to avoid this error TF_OLD_DATA ignoring data from the past for frame base_link
        # WARRNING this means that it's not the same as the odom topic and the odom topic time is probably shit
        t.header.stamp = self.get_clock().now().to_msg()
        # self.get_logger().info(f'Publishing TF from {self.frame_prefix}odom to {self.frame_prefix}base_link')
        t.header.frame_id = f"{self.frame_prefix}odom"
        t.child_frame_id = f"{self.frame_prefix}base_link"

        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z

        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomTFBroadcaster()
    rclpy.spin(node)
    rclpy.shutdown()

