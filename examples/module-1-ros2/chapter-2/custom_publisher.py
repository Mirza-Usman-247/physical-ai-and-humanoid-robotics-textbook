#!/usr/bin/env python3
"""
Custom Message Publisher
Demonstrates publishing custom message types with structured data.

Learning objectives:
- Define and use custom ROS 2 message types
- Populate composite message fields
- Configure QoS for sensor data streaming
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from std_msgs.msg import Header
import random
import time


class SensorDataPublisher(Node):
    """
    Publisher for simulated sensor data using custom message type.
    Simulates environmental sensor readings (temperature, humidity, pressure).
    """

    def __init__(self):
        super().__init__('sensor_data_publisher')

        # Sensor data QoS: Best Effort for high-frequency streaming
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Tolerate loss for performance
            durability=DurabilityPolicy.VOLATILE,       # No persistence
            history=HistoryPolicy.KEEP_LAST,
            depth=1                                      # Only keep latest reading
        )

        # Note: This assumes custom message is built and installed
        # In a real package, import would be: from my_interfaces.msg import SensorData
        # For this example, we'll use a mock structure
        self.publisher_ = self.create_publisher(
            Header,  # Placeholder - would be SensorData in real implementation
            'sensor/environmental',
            qos_profile
        )

        # Publish at 10 Hz (typical for environmental sensors)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.sensor_id = "ENV_SENSOR_001"
        self.sensor_type = "BME280"
        self.is_calibrated = True

        self.get_logger().info(f'Sensor publisher started: {self.sensor_id}')

    def timer_callback(self):
        """Generate and publish simulated sensor data."""
        # In real implementation, this would be:
        # msg = SensorData()
        # msg.header.stamp = self.get_clock().now().to_msg()
        # msg.header.frame_id = 'sensor_frame'
        # msg.sensor_id = self.sensor_id
        # msg.sensor_type = self.sensor_type
        # msg.temperature = 20.0 + random.uniform(-2.0, 2.0)
        # msg.humidity = 45.0 + random.uniform(-5.0, 5.0)
        # msg.pressure = 101.3 + random.uniform(-0.5, 0.5)
        # msg.is_calibrated = self.is_calibrated
        # msg.error_code = 0
        # msg.measurement_confidence = 0.95

        # Placeholder using Header
        msg = Header()
        msg.stamp = self.get_clock().now().to_msg()
        msg.frame_id = 'sensor_frame'

        self.publisher_.publish(msg)

        # Log simulated readings
        temp = 20.0 + random.uniform(-2.0, 2.0)
        humidity = 45.0 + random.uniform(-5.0, 5.0)
        pressure = 101.3 + random.uniform(-0.5, 0.5)

        self.get_logger().info(
            f'Published: T={temp:.1f}°C, H={humidity:.1f}%, P={pressure:.2f}kPa',
            throttle_duration_sec=1.0  # Log once per second to avoid spam
        )


def main(args=None):
    rclpy.init(args=args)
    node = SensorDataPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down sensor publisher...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
