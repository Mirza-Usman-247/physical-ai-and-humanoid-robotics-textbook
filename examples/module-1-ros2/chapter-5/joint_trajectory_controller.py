#!/usr/bin/env python3
"""
ROS 2 Control - Joint Trajectory Controller Example
Demonstrates commanding a robot arm using trajectory control.

Learning objectives:
- Use ros2_control architecture (controller manager, hardware interface, controllers)
- Send joint trajectory commands to a trajectory controller
- Monitor joint states and feedback
- Understand real-time control loops in ROS 2
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import math
import time


class JointTrajectoryControllerNode(Node):
    """
    Node that sends trajectory commands to a robot arm.

    Typical ros2_control architecture:
    1. Hardware Interface: Reads joint sensors, writes joint commands
    2. Controller Manager: Manages lifecycle of controllers (load/start/stop)
    3. Joint Trajectory Controller: Commands joints to follow a trajectory
    4. This node: Sends trajectory goals to the controller
    """

    def __init__(self):
        super().__init__('joint_trajectory_controller')

        # Create action client for trajectory control
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory'
        )

        # Subscribe to joint states for monitoring
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            self.qos_profile
        )

        # Storage for current joint states
        self.current_positions = {}
        self.current_velocities = {}

        # Joint names (example for 3-DOF arm)
        self.joint_names = ['shoulder', 'elbow', 'wrist']

        # Wait for action server to become available
        self.get_logger().info('Waiting for trajectory controller...')
        self.trajectory_client.wait_for_server()
        self.get_logger().info('Trajectory controller ready!')

    def joint_state_callback(self, msg):
        """Store current joint state information."""
        for i, name in enumerate(msg.name):
            self.current_positions[name] = msg.position[i]
            self.current_velocities[name] = msg.velocity[i]

    def send_trajectory(self, goal_positions, duration_sec=3.0):
        """
        Send a trajectory goal to the controller.

        Args:
            goal_positions: Dict mapping joint names to target positions (radians)
            duration_sec: Time to reach the goal (seconds)
        """
        # Create trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        # Create a single trajectory point (goal position)
        point = JointTrajectoryPoint()
        point.positions = [goal_positions.get(name, 0.0) for name in self.joint_names]
        point.velocities = [0.0] * len(self.joint_names)  # Stop at goal
        point.accelerations = [0.0] * len(self.joint_names)  # No acceleration info

        # Set duration
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)

        trajectory.points = [point]

        # Create action goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory
        goal_msg.goal_time_tolerance = Duration(sec=1, nanosec=0)

        # Send goal and wait for result
        self.get_logger().info(f'Sending trajectory goal: {goal_positions}')
        self._send_goal_future = self.trajectory_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Called when server accepts/rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return

        self.get_logger().info('Goal accepted!')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        """Called periodically as the trajectory executes."""
        feedback = feedback_msg.feedback
        positions_str = ', '.join([
            f'{name}={pos:.2f}rad'
            for name, pos in zip(feedback.joint_names, feedback.actual.positions)
        ])
        self.get_logger().info(f'Executing: {positions_str}')

    def get_result_callback(self, future):
        """Called when goal completes."""
        result = future.result()
        if result.error_code == result.SUCCESSFUL:
            self.get_logger().info('Trajectory execution SUCCESSFUL!')
        else:
            self.get_logger().error(f'Trajectory execution failed: {result.error_code}')

    def send_demo_trajectory(self):
        """Send a simple demonstration trajectory."""
        # Goal position 1: Raise shoulder
        goal_1 = {
            'shoulder': math.pi / 4,  # 45 degrees
            'elbow': 0.0,
            'wrist': 0.0
        }
        self.send_trajectory(goal_1, duration_sec=2.0)
        time.sleep(3)  # Wait for execution + some margin

        # Goal position 2: Lower shoulder, raise elbow
        goal_2 = {
            'shoulder': math.pi / 6,  # 30 degrees
            'elbow': math.pi / 4,  # 45 degrees
            'wrist': 0.0
        }
        self.send_trajectory(goal_2, duration_sec=2.0)
        time.sleep(3)

        # Goal position 3: Return to home
        goal_home = {
            'shoulder': 0.0,
            'elbow': 0.0,
            'wrist': 0.0
        }
        self.send_trajectory(goal_home, duration_sec=2.0)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = JointTrajectoryControllerNode()

    try:
        # Execute demo trajectory
        node.send_demo_trajectory()
        # Keep node alive for callbacks
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
