#!/usr/bin/env python3
"""
Minimal Publisher Example
Demonstrates basic topic publishing with standard message types.

Learning objectives:
- Create a publisher for a standard ROS 2 message type
- Publish data at a fixed rate using timers
- Configure QoS profiles for different use cases
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import String


class MinimalPublisher(Node):
    """
    Simple publisher that sends string messages on /chatter topic.
    """

    def __init__(self):
        super().__init__('minimal_publisher')

        # Configure QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Create publisher
        self.publisher_ = self.create_publisher(
            String,
            'chatter',
            qos_profile
        )

        # Publish at 2 Hz
        self.timer_period = 0.5  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.count = 0
        self.get_logger().info('Minimal publisher started on /chatter')

    def timer_callback(self):
        """Publish message callback."""
        msg = String()
        msg.data = f'Hello World: {self.count}'
        self.publisher_.publish(msg)

        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
