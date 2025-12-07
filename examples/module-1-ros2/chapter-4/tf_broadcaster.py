#!/usr/bin/env python3
"""
TF2 Static and Dynamic Transform Broadcaster Example
Demonstrates publishing static and dynamic coordinate frame transformations.

Learning objectives:
- Create static transforms (e.g., sensor mounting)
- Publish dynamic transforms (e.g., robot movement)
- Use tf2_ros StaticTransformBroadcaster and TransformBroadcaster
- Understand coordinate frame conventions and naming
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped


class TFBroadcasterNode(Node):
    """
    Node that publishes both static and dynamic transforms.

    Static transforms: base_link → camera_link (fixed mounting)
    Dynamic transforms: odom → base_link (robot position over time)
    """

    def __init__(self):
        super().__init__('tf_broadcaster')

        # Static broadcaster for fixed transforms
        self.static_broadcaster = StaticTransformBroadcaster(self)

        # Dynamic broadcaster for moving transforms
        self.broadcaster = TransformBroadcaster(self)

        # Publish static transform once at startup
        self._publish_static_transforms()

        # Timer for dynamic transform updates (e.g., simulating robot movement)
        self.timer_period = 0.1  # 10 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # Simulation time counter
        self.counter = 0

        self.get_logger().info('TF Broadcaster node started')

    def _publish_static_transforms(self):
        """
        Publish static transforms that don't change.
        Example: Camera mounted on the robot base.
        """
        # Static transform: base_link → camera_link
        # Camera is mounted 0.1m forward, 0.05m up from robot base
        static_transform = TransformStamped()
        static_transform.header.stamp = self.get_clock().now().to_msg()
        static_transform.header.frame_id = 'base_link'
        static_transform.child_frame_id = 'camera_link'

        # Translation (x, y, z) in meters
        static_transform.transform.translation.x = 0.1
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.05

        # Rotation (quaternion: x, y, z, w) - no rotation
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0

        # Publish once
        self.static_broadcaster.sendTransform([static_transform])
        self.get_logger().info('Published static transform: base_link → camera_link')

    def timer_callback(self):
        """
        Periodically publish dynamic transforms.
        Simulates robot moving in a circular path.
        """
        # Simulate circular motion
        radius = 1.0  # 1 meter radius
        angular_velocity = 0.5  # radians per second
        time_seconds = self.counter * self.timer_period

        # Calculate position on circle
        x = radius * math.cos(angular_velocity * time_seconds)
        y = radius * math.sin(angular_velocity * time_seconds)
        theta = angular_velocity * time_seconds

        # Create transform: odom → base_link
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'odom'  # Reference frame
        transform.child_frame_id = 'base_link'  # Robot frame

        # Translation (robot position)
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0

        # Rotation (convert yaw angle to quaternion)
        # For 2D rotation around z-axis only: q = [0, 0, sin(θ/2), cos(θ/2)]
        half_theta = theta / 2.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(half_theta)
        transform.transform.rotation.w = math.cos(half_theta)

        # Publish transform
        self.broadcaster.sendTransform(transform)

        # Log occasionally to avoid spam
        if self.counter % 50 == 0:
            self.get_logger().info(
                f'Published dynamic transform: '
                f'odom → base_link (x={x:.2f}, y={y:.2f}, θ={math.degrees(theta):.1f}°)'
            )

        self.counter += 1


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = TFBroadcasterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
