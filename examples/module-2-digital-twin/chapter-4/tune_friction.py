#!/usr/bin/env python3
"""
System Identification: Tune simulation friction parameter to match real data.

Usage:
    python3 tune_friction.py --real-data real_pendulum.csv --sim-config pendulum.urdf
"""

import numpy as np
from scipy.optimize import minimize
import matplotlib.pyplot as plt


def load_real_data(filename):
    """Load real robot trajectory data."""
    # Expected format: time, angle, velocity
    data = np.loadtxt(filename, delimiter=',', skiprows=1)
    return data


def simulate_pendulum(friction_coeff, duration=5.0, dt=0.01):
    """
    Simple pendulum simulation with damping.

    Args:
        friction_coeff: Damping coefficient (N·m·s/rad)
        duration: Simulation time (seconds)
        dt: Time step (seconds)

    Returns:
        trajectory: Array of [time, angle, velocity]
    """
    g = 9.81
    L = 1.0
    m = 1.0

    # Initial conditions
    theta = np.radians(45.0)
    omega = 0.0

    trajectory = []
    t = 0.0

    while t < duration:
        # Damped pendulum dynamics
        alpha = -(g / L) * np.sin(theta) - (friction_coeff / (m * L**2)) * omega

        # Euler integration
        omega += alpha * dt
        theta += omega * dt
        t += dt

        trajectory.append([t, theta, omega])

    return np.array(trajectory)


def compute_error(sim_traj, real_traj):
    """Compute RMS error between simulated and real trajectories."""
    # Interpolate to common time points
    common_times = real_traj[:, 0]
    sim_interp = np.interp(common_times, sim_traj[:, 0], sim_traj[:, 1])
    real_angles = real_traj[:, 1]

    # RMS error in angle
    error = np.sqrt(np.mean((sim_interp - real_angles)**2))
    return error


def objective_function(params, real_traj):
    """Objective function for optimization."""
    friction_coeff = params[0]

    # Run simulation
    sim_traj = simulate_pendulum(friction_coeff, duration=5.0)

    # Compute error
    error = compute_error(sim_traj, real_traj)

    print(f"  Friction: {friction_coeff:.4f}, Error: {np.degrees(error):.2f}°")
    return error


def main():
    # Simulate "real" data with known friction for demonstration
    print("Generating synthetic 'real' data with friction=0.15...")
    real_traj = simulate_pendulum(friction_coeff=0.15, duration=5.0)

    # Add noise to simulate real sensor data
    real_traj[:, 1] += np.random.normal(0, 0.01, len(real_traj))  # ±0.01 rad noise

    # Initial guess
    initial_friction = 0.05

    print(f"\nOptimizing friction parameter (initial guess: {initial_friction})...")
    print("-" * 60)

    # Optimize
    result = minimize(
        objective_function,
        x0=[initial_friction],
        args=(real_traj,),
        bounds=[(0.01, 0.5)],
        method='L-BFGS-B'
    )

    optimal_friction = result.x[0]
    final_error = result.fun

    print("-" * 60)
    print(f"\nOptimization complete!")
    print(f"  Optimal friction: {optimal_friction:.4f} N·m·s/rad")
    print(f"  Final RMS error: {np.degrees(final_error):.2f}°")
    print(f"  True value (ground truth): 0.15 N·m·s/rad")
    print(f"  Error from truth: {abs(optimal_friction - 0.15):.4f}")

    # Plot comparison
    sim_traj_optimal = simulate_pendulum(optimal_friction, duration=5.0)

    plt.figure(figsize=(10, 6))
    plt.plot(real_traj[:, 0], np.degrees(real_traj[:, 1]),
             'b-', label='Real Data', linewidth=2)
    plt.plot(sim_traj_optimal[:, 0], np.degrees(sim_traj_optimal[:, 1]),
             'r--', label=f'Optimized Sim (friction={optimal_friction:.3f})', linewidth=2)
    plt.xlabel('Time (s)')
    plt.ylabel('Angle (degrees)')
    plt.title('System Identification: Pendulum Friction Tuning')
    plt.legend()
    plt.grid(True)
    plt.savefig('friction_tuning_result.png', dpi=150)
    print("\nPlot saved to: friction_tuning_result.png")
    plt.show()


if __name__ == '__main__':
    main()
