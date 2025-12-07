#!/usr/bin/env python3
"""
Hello Physical AI - Simple demonstration of embodied intelligence concept

This script demonstrates the basic principle of Physical AI where intelligence
emerges from the interaction between an agent and its physical environment.
"""

import time
import math
import random


class SimpleRobot:
    """A simple simulated robot to demonstrate embodied intelligence concepts."""

    def __init__(self):
        self.position = [0.0, 0.0]  # x, y coordinates
        self.orientation = 0.0      # angle in radians
        self.sensors = {
            'front_distance': 1.0,
            'left_distance': 1.0,
            'right_distance': 1.0
        }
        self.battery_level = 100.0

    def sense_environment(self):
        """Simulate sensing the environment."""
        # In a real implementation, this would interface with actual sensors
        # For simulation, we'll generate some sensor readings
        self.sensors['front_distance'] = 1.0 + 0.5 * math.sin(time.time())
        self.sensors['left_distance'] = 1.0 + 0.5 * math.sin(time.time() + 1.0)
        self.sensors['right_distance'] = 1.0 + 0.5 * math.sin(time.time() + 2.0)

        # Simulate battery drain
        self.battery_level -= 0.1

    def make_decision(self):
        """Make a decision based on sensor input - demonstrating embodied intelligence."""
        # Simple obstacle avoidance behavior
        if self.sensors['front_distance'] < 0.5:
            # Obstacle ahead, turn left
            return 'turn_left'
        elif self.sensors['left_distance'] < self.sensors['right_distance']:
            # More space on the right
            return 'turn_right'
        else:
            # Move forward
            return 'move_forward'

    def execute_action(self, action):
        """Execute the decided action."""
        if action == 'move_forward':
            self.position[0] += 0.1 * math.cos(self.orientation)
            self.position[1] += 0.1 * math.sin(self.orientation)
        elif action == 'turn_left':
            self.orientation += 0.2
        elif action == 'turn_right':
            self.orientation -= 0.2


def main():
    """Main function demonstrating the Physical AI concept."""
    print("Hello Physical AI!")
    print("Demonstrating embodied intelligence through sensorimotor interaction.\n")

    robot = SimpleRobot()

    print("Initial state:")
    print(f"  Position: {robot.position}")
    print(f"  Battery: {robot.battery_level:.1f}%")
    print(f"  Sensors: {robot.sensors}\n")

    # Run the robot for a few steps
    for step in range(10):
        print(f"Step {step + 1}:")

        # Sense the environment
        robot.sense_environment()
        print(f"  Sensing: {robot.sensors}")

        # Make a decision based on sensor input
        action = robot.make_decision()
        print(f"  Decision: {action}")

        # Execute the action
        robot.execute_action(action)
        print(f"  New position: [{robot.position[0]:.2f}, {robot.position[1]:.2f}]")
        print(f"  Battery: {robot.battery_level:.1f}%\n")

        time.sleep(0.1)  # Small delay to simulate real-time behavior

    print("Physical AI demonstration complete!")
    print("Notice how the robot's behavior emerges from the interaction")
    print("between sensing, decision-making, and acting in the environment.")


if __name__ == "__main__":
    main()