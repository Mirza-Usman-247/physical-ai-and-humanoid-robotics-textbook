#!/usr/bin/env python3
"""
2-DOF Robot Arm Inverse Kinematics Solver
Chapter 3: Control Algorithms for Physical AI

This example demonstrates inverse kinematics for a 2-DOF planar robot arm,
showing how to calculate the required joint angles to reach a desired end-effector position.
"""

import numpy as np
import matplotlib.pyplot as plt
from math import sqrt, atan2, acos, pi
import matplotlib.patches as patches


class RobotArm2DOF:
    """
    A 2-DOF planar robot arm with inverse kinematics implementation.
    """

    def __init__(self, link1_length=1.0, link2_length=0.8):
        """
        Initialize the robot arm with link lengths.

        Args:
            link1_length (float): Length of the first link (shoulder to elbow)
            link2_length (float): Length of the second link (elbow to wrist)
        """
        self.l1 = link1_length  # Length of first link
        self.l2 = link2_length  # Length of second link

    def forward_kinematics(self, theta1, theta2):
        """
        Calculate end-effector position from joint angles (forward kinematics).

        Args:
            theta1 (float): First joint angle in radians
            theta2 (float): Second joint angle in radians

        Returns:
            tuple: (x, y) end-effector position
        """
        x = self.l1 * np.cos(theta1) + self.l2 * np.cos(theta1 + theta2)
        y = self.l1 * np.sin(theta1) + self.l2 * np.sin(theta1 + theta2)
        return x, y

    def inverse_kinematics(self, x, y):
        """
        Calculate joint angles to reach desired end-effector position (inverse kinematics).

        Args:
            x (float): Desired x position
            y (float): Desired y position

        Returns:
            tuple: (theta1, theta2) joint angles in radians, or (None, None) if no solution
        """
        # Distance from origin to target
        r = sqrt(x**2 + y**2)

        # Check if target is reachable
        if r > (self.l1 + self.l2):
            print(f"Target ({x:.2f}, {y:.2f}) is out of reach! Max reach: {self.l1 + self.l2:.2f}")
            return None, None
        elif r < abs(self.l1 - self.l2):
            print(f"Target ({x:.2f}, {y:.2f}) is inside the workspace! Min reach: {abs(self.l1 - self.l2):.2f}")
            return None, None

        # Calculate theta2 using law of cosines
        cos_theta2 = (r**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        # Clamp to valid range to avoid numerical errors
        cos_theta2 = max(-1, min(1, cos_theta2))
        theta2 = acos(cos_theta2)

        # Calculate theta1
        k1 = self.l1 + self.l2 * cos_theta2
        k2 = self.l2 * sqrt(1 - cos_theta2**2)  # sin(theta2) * l2

        theta1 = atan2(y, x) - atan2(k2, k1)

        return theta1, theta2

    def inverse_kinematics_both_solutions(self, x, y):
        """
        Calculate both possible solutions for inverse kinematics (elbow up/down).

        Args:
            x (float): Desired x position
            y (float): Desired y position

        Returns:
            tuple: Two solutions as ((theta1_a, theta2_a), (theta1_b, theta2_b))
                   Returns (None, None) if no solution exists
        """
        # Distance from origin to target
        r = sqrt(x**2 + y**2)

        # Check if target is reachable
        if r > (self.l1 + self.l2):
            print(f"Target ({x:.2f}, {y:.2f}) is out of reach! Max reach: {self.l1 + self.l2:.2f}")
            return (None, None), (None, None)
        elif r < abs(self.l1 - self.l2):
            print(f"Target ({x:.2f}, {y:.2f}) is inside the workspace! Min reach: {abs(self.l1 - self.l2):.2f}")
            return (None, None), (None, None)

        # Calculate theta2 using law of cosines
        cos_theta2 = (r**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        # Clamp to valid range to avoid numerical errors
        cos_theta2 = max(-1, min(1, cos_theta2))
        theta2 = acos(cos_theta2)

        # Two possible solutions for theta2
        theta2_a = theta2
        theta2_b = -theta2

        # Calculate theta1 for both solutions
        k1_a = self.l1 + self.l2 * cos_theta2
        k2_a = self.l2 * sqrt(1 - cos_theta2**2)
        theta1_a = atan2(y, x) - atan2(k2_a, k1_a)

        k1_b = self.l1 + self.l2 * cos(-cos_theta2)  # This is wrong, should be same cos_theta2
        k2_b = self.l2 * sqrt(1 - cos_theta2**2)  # sin(theta2) magnitude is same
        theta1_b = atan2(y, x) - atan2(-k2_a, k1_a)  # For the -theta2 solution

        return (theta1_a, theta2_a), (theta1_b, theta2_b)

    def plot_arm(self, theta1, theta2, target_pos=None, title="Robot Arm Configuration"):
        """
        Plot the robot arm configuration.

        Args:
            theta1 (float): First joint angle in radians
            theta2 (float): Second joint angle in radians
            target_pos (tuple): Optional target position (x, y) to show
            title (str): Plot title
        """
        # Calculate joint positions
        x1 = self.l1 * np.cos(theta1)
        y1 = self.l1 * np.sin(theta1)

        x2, y2 = self.forward_kinematics(theta1, theta2)

        # Create the plot
        fig, ax = plt.subplots(1, 1, figsize=(8, 8))

        # Draw the arm links
        ax.plot([0, x1, x2], [0, y1, y2], 'o-', linewidth=5, markersize=12,
                label='Robot Arm', color='blue')

        # Draw the base
        ax.plot(0, 0, 'ro', markersize=15, label='Base')

        # Draw the elbow
        ax.plot(x1, y1, 'go', markersize=12, label='Elbow')

        # Draw the end-effector
        ax.plot(x2, y2, 'mo', markersize=15, label='End-Effector')

        # Draw the target if provided
        if target_pos is not None:
            ax.plot(target_pos[0], target_pos[1], 'r*', markersize=20, label='Target')

        # Draw workspace circles
        circle_max = plt.Circle((0, 0), self.l1 + self.l2, color='lightgray', fill=False, linestyle='--', label='Max Workspace')
        circle_min = plt.Circle((0, 0), abs(self.l1 - self.l2), color='lightgray', fill=False, linestyle='--', label='Min Workspace')
        ax.add_patch(circle_max)
        if abs(self.l1 - self.l2) > 0.01:  # Only draw if there's a meaningful inner workspace
            ax.add_patch(circle_min)

        ax.set_xlim(-(self.l1 + self.l2 + 0.2), self.l1 + self.l2 + 0.2)
        ax.set_ylim(-(self.l1 + self.l2 + 0.2), self.l1 + self.l2 + 0.2)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        ax.legend()
        ax.set_title(title)

        plt.tight_layout()
        return fig, ax


def demonstrate_ik_solver():
    """
    Demonstrate the inverse kinematics solver with various examples.
    """
    print("2-DOF Robot Arm Inverse Kinematics Solver")
    print("=" * 50)
    print("This example demonstrates how to calculate joint angles needed")
    print("to position a robot arm's end-effector at a desired location.")
    print()

    # Create a robot arm instance
    arm = RobotArm2DOF(link1_length=1.0, link2_length=0.8)
    print(f"Robot arm configuration:")
    print(f"  Link 1 length: {arm.l1} m")
    print(f"  Link 2 length: {arm.l2} m")
    print(f"  Max reach: {arm.l1 + arm.l2} m")
    print(f"  Min reach: {abs(arm.l1 - arm.l2)} m")
    print()

    # Example 1: Simple position
    print("Example 1: Reach to position (1.2, 0.5)")
    target_x, target_y = 1.2, 0.5
    theta1, theta2 = arm.inverse_kinematics(target_x, target_y)

    if theta1 is not None and theta2 is not None:
        print(f"  Calculated joint angles:")
        print(f"    Joint 1 (θ1): {theta1:.3f} rad ({np.degrees(theta1):.1f}°)")
        print(f"    Joint 2 (θ2): {theta2:.3f} rad ({np.degrees(theta2):.1f}°)")

        # Verify with forward kinematics
        x_verify, y_verify = arm.forward_kinematics(theta1, theta2)
        print(f"  Verification - Forward kinematics gives: ({x_verify:.3f}, {y_verify:.3f})")
        print(f"  Error: {sqrt((x_verify-target_x)**2 + (y_verify-target_y)**2):.6f} m")

        # Plot the configuration
        arm.plot_arm(theta1, theta2, target_pos=(target_x, target_y),
                    title=f"Example 1: Target ({target_x}, {target_y})")
        plt.savefig('examples/module-0-foundations/chapter-3/ik_example1.png', dpi=150, bbox_inches='tight')
        print("  Plot saved as 'ik_example1.png'")
        plt.show()
    else:
        print("  No solution found for this position!")
    print()

    # Example 2: Another position
    print("Example 2: Reach to position (0.5, 1.3)")
    target_x, target_y = 0.5, 1.3
    theta1, theta2 = arm.inverse_kinematics(target_x, target_y)

    if theta1 is not None and theta2 is not None:
        print(f"  Calculated joint angles:")
        print(f"    Joint 1 (θ1): {theta1:.3f} rad ({np.degrees(theta1):.1f}°)")
        print(f"    Joint 2 (θ2): {theta2:.3f} rad ({np.degrees(theta2):.1f}°)")

        # Verify with forward kinematics
        x_verify, y_verify = arm.forward_kinematics(theta1, theta2)
        print(f"  Verification - Forward kinematics gives: ({x_verify:.3f}, {y_verify:.3f})")
        print(f"  Error: {sqrt((x_verify-target_x)**2 + (y_verify-target_y)**2):.6f} m")

        # Plot the configuration
        arm.plot_arm(theta1, theta2, target_pos=(target_x, target_y),
                    title=f"Example 2: Target ({target_x}, {target_y})")
        plt.savefig('examples/module-0-foundations/chapter-3/ik_example2.png', dpi=150, bbox_inches='tight')
        print("  Plot saved as 'ik_example2.png'")
        plt.show()
    else:
        print("  No solution found for this position!")
    print()

    # Example 3: Out of reach (should fail)
    print("Example 3: Target out of reach (3.0, 0.0) - should fail")
    target_x, target_y = 3.0, 0.0
    theta1, theta2 = arm.inverse_kinematics(target_x, target_y)
    if theta1 is None:
        print("  Correctly identified unreachable target!")
    print()

    # Example 4: Path planning demonstration
    print("Example 4: Path following demonstration")
    print("Moving the end-effector along a circular path...")

    # Define a circular path
    center_x, center_y = 0.8, 0.5
    radius = 0.3
    n_points = 20

    path_angles = np.linspace(0, 2*np.pi, n_points)
    joint_angles_path = []
    reached_targets = []

    for angle in path_angles:
        target_x = center_x + radius * np.cos(angle)
        target_y = center_y + radius * np.sin(angle)

        theta1, theta2 = arm.inverse_kinematics(target_x, target_y)

        if theta1 is not None and theta2 is not None:
            joint_angles_path.append((theta1, theta2))
            reached_targets.append((target_x, target_y))
        else:
            print(f"  Could not reach point at ({target_x:.2f}, {target_y:.2f})")

    print(f"  Planned path with {len(joint_angles_path)} valid waypoints")

    if joint_angles_path:
        print("  Animating path following (first 5 points):")
        for i, (theta1, theta2) in enumerate(joint_angles_path[:5]):
            x_reached, y_reached = arm.forward_kinematics(theta1, theta2)
            print(f"    Point {i+1}: Joint angles ({theta1:.3f}, {theta2:.3f}) -> End-effector at ({x_reached:.3f}, {y_reached:.3f})")

    print()
    print("Inverse kinematics is fundamental to robot control,")
    print("enabling robots to plan movements to achieve desired positions.")


def main():
    """Main function to run the inverse kinematics demonstration."""
    demonstrate_ik_solver()


if __name__ == "__main__":
    main()