#!/usr/bin/env python3
"""
Simple Perception-Action Loop Simulation
Chapter 2: Robotics Simulation Fundamentals

This example demonstrates a basic perception-action loop where a simulated robot
perceives its environment and acts based on that perception, implementing the
core concept of embodied intelligence in simulation environments.
"""

import numpy as np
import matplotlib.pyplot as plt
import time
import math
from dataclasses import dataclass
from typing import Tuple, List


@dataclass
class RobotState:
    """Represents the state of our simulated robot."""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0  # orientation in radians
    velocity: float = 0.0
    angular_velocity: float = 0.0


@dataclass
class SensorData:
    """Represents sensor data from the robot."""
    distance_to_goal: float
    angle_to_goal: float
    obstacles: List[Tuple[float, float]]  # List of (distance, angle) to obstacles
    proximity_sensors: List[float]  # Front, left, right proximity readings


class Environment:
    """A simple 2D environment for our robot to navigate."""

    def __init__(self, width: float = 10.0, height: float = 10.0):
        self.width = width
        self.height = height
        self.goal_position = np.array([width * 0.8, height * 0.8])  # Goal at 80% of width/height
        self.obstacles = [
            np.array([width * 0.5, height * 0.3]),  # Obstacle 1
            np.array([width * 0.3, height * 0.6]),  # Obstacle 2
            np.array([width * 0.7, height * 0.5])   # Obstacle 3
        ]

    def get_sensor_data(self, robot_state: RobotState) -> SensorData:
        """Simulate sensor readings for the robot."""
        robot_pos = np.array([robot_state.x, robot_state.y])

        # Calculate distance and angle to goal
        to_goal = self.goal_position - robot_pos
        distance_to_goal = np.linalg.norm(to_goal)
        angle_to_goal = math.atan2(to_goal[1], to_goal[0]) - robot_state.theta

        # Normalize angle to [-pi, pi]
        angle_to_goal = math.atan2(math.sin(angle_to_goal), math.cos(angle_to_goal))

        # Simulate proximity sensors (front, left, right)
        proximity_readings = []
        sensor_angles = [0, math.pi/2, -math.pi/2]  # Front, left, right

        for sensor_angle in sensor_angles:
            sensor_world_angle = robot_state.theta + sensor_angle
            # Simulate ray casting to detect obstacles
            min_distance = 2.0  # Max sensor range

            for obstacle in self.obstacles:
                to_obstacle = obstacle - robot_pos
                obstacle_distance = np.linalg.norm(to_obstacle)
                obstacle_angle = math.atan2(to_obstacle[1], to_obstacle[0]) - sensor_world_angle
                obstacle_angle = math.atan2(math.sin(obstacle_angle), math.cos(obstacle_angle))

                # Check if obstacle is within sensor field of view (±30 degrees)
                if abs(obstacle_angle) < math.pi / 6 and obstacle_distance < min_distance:
                    min_distance = obstacle_distance

            proximity_readings.append(min_distance)

        # Calculate obstacle distances and angles
        obstacle_data = []
        for obstacle in self.obstacles:
            to_obstacle = obstacle - robot_pos
            obstacle_distance = np.linalg.norm(to_obstacle)
            obstacle_angle = math.atan2(to_obstacle[1], to_obstacle[0]) - robot_state.theta
            obstacle_angle = math.atan2(math.sin(obstacle_angle), math.cos(obstacle_angle))
            obstacle_data.append((obstacle_distance, obstacle_angle))

        return SensorData(
            distance_to_goal=distance_to_goal,
            angle_to_goal=angle_to_goal,
            obstacles=obstacle_data,
            proximity_sensors=proximity_readings
        )

    def check_collision(self, robot_state: RobotState) -> bool:
        """Check if robot collides with any obstacles."""
        robot_pos = np.array([robot_state.x, robot_state.y])
        robot_radius = 0.3  # Robot collision radius

        for obstacle in self.obstacles:
            if np.linalg.norm(robot_pos - obstacle) < robot_radius:
                return True
        return False


class PerceptionActionController:
    """Implements the perception-action loop for the robot."""

    def __init__(self, max_linear_speed: float = 1.0, max_angular_speed: float = 1.0):
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.goal_tolerance = 0.5  # Distance threshold to consider goal reached

    def perceive(self, sensor_data: SensorData) -> dict:
        """Process sensor data to understand the environment."""
        return {
            'at_goal': sensor_data.distance_to_goal < self.goal_tolerance,
            'obstacle_front': sensor_data.proximity_sensors[0] < 0.8,
            'obstacle_left': sensor_data.proximity_sensors[1] < 0.8,
            'obstacle_right': sensor_data.proximity_sensors[2] < 0.8,
            'goal_direction': sensor_data.angle_to_goal,
            'goal_distance': sensor_data.distance_to_goal
        }

    def act(self, perception: dict) -> Tuple[float, float]:
        """Determine actions based on perception."""
        if perception['at_goal']:
            return 0.0, 0.0  # Stop when at goal

        linear_vel = 0.0
        angular_vel = 0.0

        # Simple obstacle avoidance and goal-seeking behavior
        if perception['obstacle_front']:
            # Obstacle ahead - turn away
            if perception['obstacle_left'] and not perception['obstacle_right']:
                angular_vel = -self.max_angular_speed * 0.5  # Turn right
            elif perception['obstacle_right'] and not perception['obstacle_left']:
                angular_vel = self.max_angular_speed * 0.5   # Turn left
            else:
                # Both sides blocked or neither - turn randomly
                angular_vel = self.max_angular_speed * 0.3 * np.sign(perception['goal_direction'])
        else:
            # No obstacle ahead - move toward goal
            linear_vel = self.max_linear_speed
            angular_vel = self.max_angular_speed * 0.5 * perception['goal_direction']

        # Clamp velocities
        linear_vel = np.clip(linear_vel, 0, self.max_linear_speed)
        angular_vel = np.clip(angular_vel, -self.max_angular_speed, self.max_angular_speed)

        return linear_vel, angular_vel


def simulate_robot():
    """Run the perception-action loop simulation."""
    print("Starting Perception-Action Loop Simulation")
    print("Robot will navigate to goal while avoiding obstacles using sensor feedback.")
    print()

    # Initialize environment and robot
    env = Environment()
    robot = RobotState(x=1.0, y=1.0, theta=0.0)
    controller = PerceptionActionController()

    # For visualization
    robot_positions = [(robot.x, robot.y)]
    goal_positions = [env.goal_position]
    obstacle_positions = env.obstacles

    # Simulation parameters
    dt = 0.1  # Time step
    max_steps = 500  # Maximum simulation steps
    step = 0

    print(f"Initial position: ({robot.x:.2f}, {robot.y:.2f})")
    print(f"Goal position: ({env.goal_position[0]:.2f}, {env.goal_position[1]:.2f})")
    print()

    try:
        while step < max_steps:
            # PERCEPTION: Get sensor data
            sensor_data = env.get_sensor_data(robot)

            # PROCESS: Convert sensor data to meaningful perception
            perception = controller.perceive(sensor_data)

            # ACTION: Determine movement based on perception
            linear_vel, angular_vel = controller.act(perception)

            # PHYSICS: Update robot state based on actions
            robot.x += linear_vel * math.cos(robot.theta) * dt
            robot.y += linear_vel * math.sin(robot.theta) * dt
            robot.theta += angular_vel * dt

            # Keep robot within bounds
            robot.x = np.clip(robot.x, 0.1, env.width - 0.1)
            robot.y = np.clip(robot.y, 0.1, env.height - 0.1)

            # Record position for visualization
            robot_positions.append((robot.x, robot.y))

            # Check for collisions
            if env.check_collision(robot):
                print(f"Collision detected at step {step}! Position: ({robot.x:.2f}, {robot.y:.2f})")
                break

            # Check if goal reached
            if perception['at_goal']:
                print(f"Goal reached at step {step}! Position: ({robot.x:.2f}, {robot.y:.2f})")
                break

            # Print status every 50 steps
            if step % 50 == 0:
                print(f"Step {step}: Pos=({robot.x:.2f}, {robot.y:.2f}), "
                      f"Dist to goal={sensor_data.distance_to_goal:.2f}, "
                      f"Linear vel={linear_vel:.2f}, Angular vel={angular_vel:.2f}")

            step += 1

        if step >= max_steps and not perception.get('at_goal', False):
            print(f"Max steps reached without reaching goal. Final position: ({robot.x:.2f}, {robot.y:.2f})")

        print()
        print("Simulation complete!")
        print(f"Total steps: {step}")
        print(f"Final distance to goal: {np.linalg.norm(np.array([robot.x, robot.y]) - env.goal_position):.2f}")

        # Visualize the path
        visualize_path(robot_positions, goal_positions, obstacle_positions)

    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")


def visualize_path(robot_positions, goal_positions, obstacle_positions):
    """Create a visualization of the robot's path."""
    robot_x, robot_y = zip(*robot_positions)
    goal_x, goal_y = goal_positions[0]
    obs_x = [obs[0] for obs in obstacle_positions]
    obs_y = [obs[1] for obs in obstacle_positions]

    plt.figure(figsize=(10, 8))
    plt.plot(robot_x, robot_y, 'b-', linewidth=2, label='Robot Path')
    plt.plot(robot_x[0], robot_y[0], 'go', markersize=10, label='Start')
    plt.plot(robot_x[-1], robot_y[-1], 'ro', markersize=10, label='End')
    plt.plot(goal_x, goal_y, 'gs', markersize=12, label='Goal')
    plt.scatter(obs_x, obs_y, c='red', s=200, marker='s', label='Obstacles')

    plt.title('Perception-Action Loop Simulation: Robot Navigation')
    plt.xlabel('X Position (m)')
    plt.ylabel('Y Position (m)')
    plt.legend()
    plt.grid(True, alpha=0.3)
    plt.axis('equal')

    # Save the plot
    plt.savefig('examples/module-0-foundations/chapter-2/perception_action_path.png', dpi=150, bbox_inches='tight')
    print("Path visualization saved to 'perception_action_path.png'")


def main():
    """Main function to run the perception-action loop demonstration."""
    print("Perception-Action Loop Simulation")
    print("=" * 40)
    print("This example demonstrates the core concept of embodied intelligence:")
    print("An agent (robot) perceives its environment, processes the information,")
    print("and takes actions based on that perception, creating a continuous loop.")
    print()

    simulate_robot()

    print()
    print("Key concepts demonstrated:")
    print("- Sensorimotor integration (perception → action)")
    print("- Reactive behavior based on environmental feedback")
    print("- Obstacle avoidance and goal-seeking behaviors")
    print("- Simulation as a tool for testing embodied intelligence")


if __name__ == "__main__":
    main()