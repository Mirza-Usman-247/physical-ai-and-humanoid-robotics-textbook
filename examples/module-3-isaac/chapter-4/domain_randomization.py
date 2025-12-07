#!/usr/bin/env python3
"""
Domain Randomization with Isaac Sim Replicator
Demonstrates physics and visual randomization for sim-to-real transfer.

Requirements:
    - Isaac Sim 4.0+ installed
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.replicator.core as rep
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import get_current_stage
import numpy as np


def setup_randomized_scene():
    """Create scene with domain randomization enabled."""

    # Initialize World
    world = World()
    world.scene.add_default_ground_plane()

    # Add objects to randomize
    cube_positions = [
        (0.5, 0.0, 0.5),
        (-0.5, 0.0, 0.5),
        (0.0, 0.5, 0.5),
    ]

    for i, pos in enumerate(cube_positions):
        cube = world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/Cube_{i}",
                name=f"cube_{i}",
                position=np.array(pos),
                scale=np.array([0.1, 0.1, 0.1]),
                color=np.array([1.0, 0.0, 0.0])
            )
        )

    return world


def configure_domain_randomization():
    """Configure physics and visual randomization."""

    print("Configuring domain randomization...")

    # Get all dynamic objects
    cube_prims = [f"/World/Cube_{i}" for i in range(3)]

    with rep.trigger.on_frame(num_frames=1000):
        # Physics randomization
        with rep.create.group(cube_prims):
            # Randomize mass
            rep.randomizer.physics_properties(
                mass=rep.distribution.uniform(0.5, 2.0)
            )

            # Randomize friction
            rep.randomizer.physics_properties(
                static_friction=rep.distribution.uniform(0.3, 1.5),
                dynamic_friction=rep.distribution.uniform(0.2, 1.2)
            )

            # Randomize initial position
            rep.modify.pose(
                position=rep.distribution.uniform(
                    (-0.8, -0.8, 0.3),
                    (0.8, 0.8, 0.8)
                )
            )

        # Visual randomization - Lighting
        rep.randomizer.light(
            intensity=rep.distribution.uniform(500, 2500),
            temperature=rep.distribution.uniform(3000, 8000),
            angle=rep.distribution.uniform(0, 360)
        )

        # Visual randomization - Materials
        rep.randomizer.materials(
            textures=rep.distribution.choice([
                "/Materials/Wood",
                "/Materials/Metal",
                "/Materials/Plastic",
                "/Materials/Concrete"
            ])
        )

        # Visual randomization - Colors
        rep.randomizer.color(
            colors=rep.distribution.uniform((0.1, 0.1, 0.1), (0.9, 0.9, 0.9)),
            roughness=rep.distribution.uniform(0.2, 0.9),
            metallic=rep.distribution.uniform(0.0, 0.8)
        )

    print("Domain randomization configured:")
    print("  - Physics: mass, friction, initial pose")
    print("  - Lighting: intensity, temperature, angle")
    print("  - Materials: textures, colors, roughness, metallic")


def main():
    """Run simulation with domain randomization."""

    print("Setting up randomized scene...")
    world = setup_randomized_scene()

    # Configure randomization
    configure_domain_randomization()

    # Reset world
    print("Resetting world...")
    world.reset()

    # Run orchestrator
    rep.orchestrator.run()

    # Simulation loop
    print("Running simulation with domain randomization...")
    print("Watch physics and visuals randomize each episode!\n")

    frame_count = 0
    episode = 0

    while simulation_app.is_running() and episode < 10:
        world.step(render=True)
        frame_count += 1

        # Reset every 100 frames to see randomization
        if frame_count % 100 == 0:
            world.reset()
            episode += 1
            print(f"Episode {episode}: Physics and visuals randomized")

    print("\nDomain randomization demonstration complete!")
    print("\nKey takeaways:")
    print("  1. Physics randomization makes policies robust to model errors")
    print("  2. Visual randomization enables sim-to-real for vision tasks")
    print("  3. Isaac Sim Replicator automates the randomization workflow")

    simulation_app.close()


if __name__ == "__main__":
    main()
