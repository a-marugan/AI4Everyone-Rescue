#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from unitree_slam_interface.msg import QtCommand
from std_msgs.msg import String

class SlamMappingClient(Node):
    def __init__(self):
        super().__init__('slam_mapping_client')

        # Publisher to the 'rt/qt_command' topic
        self.publisher_ = self.create_publisher(QtCommand, 'rt/qt_command', 10)

        # Subscriber to the 'rt/qt_notice' topic
        self.subscription = self.create_subscription(
            String,
            'rt/qt_notice',
            self.notice_callback,
            10
        )

        # Start mapping
        self.send_command(command=3, seq='start_mapping')

        # Wait for user input to end mapping
        input("Press Enter to stop mapping...")

        # End mapping
        self.send_command(command=4, seq='end_mapping')

    def send_command(self, command, seq):
        msg = QtCommand()
        msg.command = bytes([command])
        msg.seq = seq

        # Set other required fields if necessary
        # For mapping commands, other fields can be left as default

        self.publisher_.publish(msg)
        self.get_logger().info(f'Sent command {command} with seq "{seq}"')

    def notice_callback(self, msg):
        self.get_logger().info(f'Received notice: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    slam_client = SlamMappingClient()
    rclpy.spin(slam_client)
    slam_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
