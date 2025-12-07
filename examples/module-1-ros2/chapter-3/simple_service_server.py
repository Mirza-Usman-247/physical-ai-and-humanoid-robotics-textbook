#!/usr/bin/env python3
"""
Simple Service Server Example
Demonstrates synchronous request-reply pattern for discrete operations.

Learning objectives:
- Create a service server for on-demand computation
- Handle request parameters and return responses
- Implement error handling in services
"""

import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts


class SimpleServiceServer(Node):
    """Service server that adds two integers."""

    def __init__(self):
        super().__init__('simple_service_server')

        self.srv = self.create_service(
            AddTwoInts,
            'add_two_ints',
            self.add_two_ints_callback
        )

        self.get_logger().info('Service server ready: add_two_ints')

    def add_two_ints_callback(self, request, response):
        """
        Service callback - executes synchronously when client requests.

        Args:
            request: AddTwoInts.Request with fields a, b
            response: AddTwoInts.Response with field sum

        Returns:
            response object with computed sum
        """
        response.sum = request.a + request.b
        self.get_logger().info(f'Request: {request.a} + {request.b} = {response.sum}')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimpleServiceServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down service server...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
