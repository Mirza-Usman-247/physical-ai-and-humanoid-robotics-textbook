#!/usr/bin/env python3
"""
Simple Service Client Example
Demonstrates calling ROS 2 services synchronously.
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts
import sys


class SimpleServiceClient(Node):
    """Service client that requests integer addition."""

    def __init__(self):
        super().__init__('simple_service_client')

        self.client = self.create_client(AddTwoInts, 'add_two_ints')

        # Wait for service to become available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for service add_two_ints...')

        self.get_logger().info('Connected to add_two_ints service')

    def send_request(self, a, b):
        """Send service request and wait for response."""
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        self.get_logger().info(f'Sending request: {a} + {b}')
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'Result: {response.sum}')
            return response.sum
        else:
            self.get_logger().error('Service call failed')
            return None


def main(args=None):
    rclpy.init(args=args)

    if len(sys.argv) < 3:
        print('Usage: simple_service_client.py <a> <b>')
        return

    a = int(sys.argv[1])
    b = int(sys.argv[2])

    client_node = SimpleServiceClient()
    result = client_node.send_request(a, b)

    client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
