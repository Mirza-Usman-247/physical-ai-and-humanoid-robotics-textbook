#!/usr/bin/env python3
"""
Simple ROS 2 Node Example
Demonstrates basic node creation, lifecycle, and logging.

Learning objectives:
- Create a minimal ROS 2 node
- Understand node lifecycle
- Use ROS 2 logging system
- Clean shutdown with signal handling
"""

import rclpy
from rclpy.node import Node


class SimpleNode(Node):
    """
    A minimal ROS 2 node that demonstrates core concepts.

    This node:
    - Initializes with a custom name
    - Uses a timer callback to periodically log messages
    - Demonstrates proper logging practices
    - Shows clean shutdown handling
    """

    def __init__(self):
        super().__init__('simple_node')

        # Create a timer that triggers every 1 second
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Counter to track callback invocations
        self.counter = 0

        self.get_logger().info('Simple node has been started!')

    def timer_callback(self):
        """
        Timer callback executed every second.
        Demonstrates logging and state management.
        """
        self.counter += 1
        self.get_logger().info(f'Timer callback #{self.counter}')

        # Demonstrate different log levels
        if self.counter == 5:
            self.get_logger().warn('This is a warning at count 5')
        elif self.counter == 10:
            self.get_logger().error('This is an error message at count 10')


def main(args=None):
    """
    Main entry point for the node.
    Handles initialization and shutdown.
    """
    # Initialize the ROS 2 Python client library
    rclpy.init(args=args)

    # Create the node
    node = SimpleNode()

    try:
        # Keep the node running (blocks until shutdown)
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, shutting down...')
    finally:
        # Clean shutdown
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
