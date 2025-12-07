#!/usr/bin/env python3
"""
Simple Action Server Example
Demonstrates long-running tasks with feedback and cancellation.

Learning objectives:
- Create action server for asynchronous operations
- Provide periodic feedback during execution
- Handle goal cancellation gracefully
"""

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from example_interfaces.action import Fibonacci
import time


class SimpleActionServer(Node):
    """Action server that computes Fibonacci sequence with feedback."""

    def __init__(self):
        super().__init__('simple_action_server')

        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback
        )

        self.get_logger().info('Action server ready: fibonacci')

    def execute_callback(self, goal_handle):
        """
        Execute Fibonacci computation with periodic feedback.

        Args:
            goal_handle: Handle to track goal status and send feedback

        Returns:
            Result message with final sequence
        """
        self.get_logger().info(f'Executing goal: compute Fibonacci({goal_handle.request.order})')

        # Initialize feedback and result
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        # Compute Fibonacci sequence
        for i in range(1, goal_handle.request.order):
            # Check if goal was cancelled
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal cancelled')
                return Fibonacci.Result()

            # Compute next Fibonacci number
            feedback_msg.sequence.append(
                feedback_msg.sequence[i] + feedback_msg.sequence[i-1]
            )

            # Publish feedback
            self.get_logger().info(f'Feedback: {feedback_msg.sequence}')
            goal_handle.publish_feedback(feedback_msg)

            # Simulate computation time
            time.sleep(1.0)

        # Mark goal as succeeded
        goal_handle.succeed()

        # Return result
        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        self.get_logger().info(f'Goal succeeded: {result.sequence}')
        return result


def main(args=None):
    rclpy.init(args=args)
    node = SimpleActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down action server...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
