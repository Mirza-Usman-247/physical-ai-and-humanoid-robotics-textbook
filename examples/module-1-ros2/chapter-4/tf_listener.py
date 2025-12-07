#!/usr/bin/env python3
"""
TF2 Transform Listener Example
Demonstrates listening to and querying coordinate frame transformations.

Learning objectives:
- Listen to transforms published by broadcasters
- Query transform between two frames
- Handle timing and buffering
- Compute frame relationships for sensor fusion
"""

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import TransformListener, Buffer
from tf2_ros.transform_listener import TransformListener as TFListener
from geometry_msgs.msg import PointStamped
import math


class TFListenerNode(Node):
    """
    Node that listens to transforms and queries frame relationships.

    Demonstrates:
    - Listening to transforms published by tf_broadcaster
    - Querying specific transforms between frames
    - Using TF buffers for temporal interpolation
    - Computing relative positions/orientations
    """

    def __init__(self):
        super().__init__('tf_listener')

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TFListener(self.tf_buffer, self)

        # Timer to query transforms
        self.timer_period = 1.0  # Query every 1 second
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('TF Listener node started (waiting for transforms...)')

    def timer_callback(self):
        """Periodically query and display transforms."""
        # Wait a bit for transforms to be available
        try:
            # Query 1: Get robot position in odometry frame
            transform_odom_to_base = self.tf_buffer.lookup_transform(
                'odom',  # Target frame (reference)
                'base_link',  # Source frame (robot)
                Time()  # Latest available time
            )
            self._log_transform('odom → base_link', transform_odom_to_base)

            # Query 2: Get camera position in odometry frame
            # This requires looking up: odom → base_link → camera_link
            transform_odom_to_camera = self.tf_buffer.lookup_transform(
                'odom',
                'camera_link',
                Time()
            )
            self._log_transform('odom → camera_link (via base_link)', transform_odom_to_camera)

            # Query 3: Transform a point from camera frame to odom frame
            self._transform_point_example()

        except Exception as e:
            self.get_logger().warn(f'Transform lookup failed: {e}')

    def _log_transform(self, transform_name, transform):
        """Helper to log transform information."""
        t = transform.transform.translation
        r = transform.transform.rotation

        # Convert quaternion to yaw angle (simplified for 2D)
        yaw_rad = math.atan2(
            2 * (r.w * r.z + r.x * r.y),
            1 - 2 * (r.y * r.y + r.z * r.z)
        )
        yaw_deg = math.degrees(yaw_rad)

        self.get_logger().info(
            f'{transform_name}: '
            f'x={t.x:.2f}m, y={t.y:.2f}m, z={t.z:.2f}m, '
            f'yaw={yaw_deg:.1f}°'
        )

    def _transform_point_example(self):
        """
        Example: Transform a point from camera frame to odometry frame.
        Useful for sensor fusion (e.g., detected object in camera frame).
        """
        try:
            # Create a point in camera frame (e.g., object detected 2m ahead)
            point_stamped = PointStamped()
            point_stamped.header.frame_id = 'camera_link'
            point_stamped.header.stamp = self.get_clock().now().to_msg()
            point_stamped.point.x = 2.0  # 2 meters forward in camera view
            point_stamped.point.y = 0.0
            point_stamped.point.z = 0.0

            # Transform to odometry frame
            transformed_point = self.tf_buffer.transform(point_stamped, 'odom')

            self.get_logger().info(
                f'Point in camera_link (2.0m forward) → odom: '
                f'x={transformed_point.point.x:.2f}m, '
                f'y={transformed_point.point.y:.2f}m, '
                f'z={transformed_point.point.z:.2f}m'
            )
        except Exception as e:
            self.get_logger().debug(f'Point transformation failed: {e}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = TFListenerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt, shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
