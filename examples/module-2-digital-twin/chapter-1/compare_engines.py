#!/usr/bin/env python3
"""
Physics Engine Comparison Tool
Compares accuracy and performance across different physics engines.

Usage:
    python3 compare_engines.py --engines ode,bullet,mujoco --duration 5.0
"""

import argparse
import time
import numpy as np
import matplotlib.pyplot as plt


class PendulumSimulator:
    """Abstract base class for physics engine wrappers."""

    def __init__(self, mass=1.0, length=1.0, dt=0.001):
        self.mass = mass
        self.length = length
        self.dt = dt
        self.time = 0.0
        self.trajectory = []

    def reset(self, angle_deg=45.0):
        """Reset pendulum to initial angle (degrees)."""
        raise NotImplementedError

    def step(self):
        """Advance simulation by one timestep."""
        raise NotImplementedError

    def get_angle(self):
        """Get current pendulum angle (radians)."""
        raise NotImplementedError

    def analytical_solution(self, t):
        """Ground truth: analytical solution for small angles."""
        theta_0 = np.deg2rad(45.0)
        omega = np.sqrt(9.81 / self.length)
        return theta_0 * np.cos(omega * t)


class ODEPendulum(PendulumSimulator):
    """ODE (Open Dynamics Engine) implementation."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # Placeholder - requires pyode installation
        print("ODE engine initialized (placeholder)")

    def reset(self, angle_deg=45.0):
        self.angle = np.deg2rad(angle_deg)
        self.velocity = 0.0
        self.time = 0.0

    def step(self):
        # Simple forward Euler for demonstration
        g = 9.81
        angular_acc = -(g / self.length) * np.sin(self.angle)
        self.velocity += angular_acc * self.dt
        self.angle += self.velocity * self.dt
        self.time += self.dt

    def get_angle(self):
        return self.angle


class BulletPendulum(PendulumSimulator):
    """Bullet (PyBullet) implementation."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        print("Bullet engine initialized (placeholder)")

    def reset(self, angle_deg=45.0):
        self.angle = np.deg2rad(angle_deg)
        self.velocity = 0.0
        self.time = 0.0

    def step(self):
        # Simple forward Euler for demonstration
        g = 9.81
        angular_acc = -(g / self.length) * np.sin(self.angle)
        self.velocity += angular_acc * self.dt
        self.angle += self.velocity * self.dt
        self.time += self.dt

    def get_angle(self):
        return self.angle


def run_comparison(engines, duration=5.0, dt=0.001):
    """Run pendulum simulation across multiple engines."""

    results = {}

    for engine_name in engines:
        print(f"Running {engine_name} simulation...")

        if engine_name == "ode":
            sim = ODEPendulum(dt=dt)
        elif engine_name == "bullet":
            sim = BulletPendulum(dt=dt)
        else:
            print(f"Unknown engine: {engine_name}")
            continue

        sim.reset(angle_deg=45.0)

        times = []
        angles = []

        start_time = time.time()
        steps = int(duration / dt)

        for i in range(steps):
            sim.step()
            if i % 10 == 0:  # Record every 10 steps
                times.append(sim.time)
                angles.append(sim.get_angle())

        elapsed = time.time() - start_time

        # Compute analytical solution
        analytical_angles = [sim.analytical_solution(t) for t in times]

        # Compute RMS error
        errors = np.array(angles) - np.array(analytical_angles)
        rms_error = np.sqrt(np.mean(errors**2))

        results[engine_name] = {
            'times': times,
            'angles': angles,
            'analytical': analytical_angles,
            'rms_error_deg': np.rad2deg(rms_error),
            'sim_time_ms': elapsed * 1000,
            'real_time_factor': duration / elapsed
        }

        print(f"  RMS Error: {results[engine_name]['rms_error_deg']:.2f}°")
        print(f"  Sim Time: {results[engine_name]['sim_time_ms']:.1f} ms")
        print(f"  Real-time Factor: {results[engine_name]['real_time_factor']:.1f}x")

    return results


def plot_results(results):
    """Generate comparison plots."""

    fig, axes = plt.subplots(2, 1, figsize=(10, 8))

    # Plot trajectories
    for engine_name, data in results.items():
        axes[0].plot(data['times'], np.rad2deg(data['angles']),
                     label=f"{engine_name} (RMS: {data['rms_error_deg']:.2f}°)")

    # Plot analytical solution
    first_engine = list(results.values())[0]
    axes[0].plot(first_engine['times'], np.rad2deg(first_engine['analytical']),
                 'k--', label='Analytical', linewidth=2)

    axes[0].set_xlabel('Time (s)')
    axes[0].set_ylabel('Angle (degrees)')
    axes[0].set_title('Pendulum Trajectory Comparison')
    axes[0].legend()
    axes[0].grid(True)

    # Plot performance metrics
    engines = list(results.keys())
    rms_errors = [results[e]['rms_error_deg'] for e in engines]
    sim_times = [results[e]['sim_time_ms'] for e in engines]

    x = np.arange(len(engines))
    width = 0.35

    axes[1].bar(x - width/2, rms_errors, width, label='RMS Error (deg)')
    axes[1].set_ylabel('RMS Error (deg)')
    axes[1].set_xlabel('Physics Engine')
    axes[1].set_xticks(x)
    axes[1].set_xticklabels(engines)
    axes[1].legend()
    axes[1].grid(True, axis='y')

    plt.tight_layout()
    plt.savefig('pendulum_comparison.png', dpi=150)
    print("Plot saved to: pendulum_comparison.png")
    plt.show()


def main():
    parser = argparse.ArgumentParser(description='Compare physics engines')
    parser.add_argument('--engines', type=str, default='ode,bullet',
                       help='Comma-separated list of engines (ode,bullet,mujoco)')
    parser.add_argument('--duration', type=float, default=5.0,
                       help='Simulation duration in seconds')
    parser.add_argument('--dt', type=float, default=0.001,
                       help='Time step in seconds')

    args = parser.parse_args()
    engines = args.engines.split(',')

    results = run_comparison(engines, args.duration, args.dt)
    plot_results(results)


if __name__ == '__main__':
    main()
