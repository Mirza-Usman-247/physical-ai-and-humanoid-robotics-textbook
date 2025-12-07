#!/usr/bin/env python3
"""
Minimal Subscriber Example
Demonstrates basic topic subscription with callback handling.

Learning objectives:
- Create a subscriber for a standard ROS 2 message type
- Handle incoming messages in callback function
- Match QoS profiles between publisher and subscriber
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String


class MinimalSubscriber(Node):
    """
    Simple subscriber that receives string messages from /chatter topic.
    """

    def __init__(self):
        super().__init__('minimal_subscriber')

        # Configure QoS profile to match publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create subscriber
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            qos_profile
        )

        self.get_logger().info('Minimal subscriber started, listening to /chatter')

    def listener_callback(self, msg):
        """Callback function executed when message is received."""
        self.get_logger().info(f'I heard: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = MinimalSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down subscriber...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
