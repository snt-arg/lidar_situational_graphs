#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-2-Clause
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class Map2OdomPublisher(Node):

    def __init__(self):
        super().__init__('map2odom_publisher_node')

        self.tf_broadcaster = TransformBroadcaster(self)
        self.subcription = self.create_subscription(
            TransformStamped,
            '/s_graphs/odom2map',
            self.callback,
            qos_profile_sensor_data)

        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('map_frame_id', 'map')

        self.odom_frame_id = self.get_parameter(
            'odom_frame_id').get_parameter_value().string_value
        self.map_frame_id = self.get_parameter(
            'map_frame_id').get_parameter_value().string_value

        self.timer = self.create_timer(0.1, self.timer_callback)

    def callback(self, odom_msg):
        self.odom_msg = odom_msg

    def timer_callback(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.map_frame_id
        transform.child_frame_id = self.odom_frame_id

        if not hasattr(self, 'odom_msg'):
            transform.transform.translation.x = 0.0
            transform.transform.translation.y = 0.0
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = 0.0
            transform.transform.rotation.w = 1.0

            self.tf_broadcaster.sendTransform(transform)
            return

        transform.transform = self.odom_msg.transform
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = Map2OdomPublisher()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
