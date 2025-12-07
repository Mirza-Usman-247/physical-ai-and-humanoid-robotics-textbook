#!/usr/bin/env python3
"""
Isaac Sim Basic Scene Creation
Demonstrates USD workflow, robot import, and simulation loop.

Requirements:
    - Isaac Sim 4.0+ installed
    - NVIDIA GPU with RTX support
"""

from isaacsim import SimulationApp

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False, "width": 1280, "height": 720})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, GroundPlane
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

def create_isaac_scene():
    """Create a basic Isaac Sim scene with robot and objects."""

    # Initialize World (handles simulation lifecycle)
    world = World(stage_units_in_meters=1.0)

    # Add ground plane
    world.scene.add_default_ground_plane()
    print("Ground plane added")

    # Get Nucleus assets path
    assets_root = get_assets_root_path()
    if assets_root is None:
        print("Warning: Nucleus server not found. Using local assets.")
        robot_usd_path = "franka.usd"  # Use local file
    else:
        robot_usd_path = assets_root + "/Isaac/Robots/Franka/franka.usd"

    # Add Franka robot from USD
    try:
        add_reference_to_stage(usd_path=robot_usd_path, prim_path="/World/Franka")
        print(f"Robot loaded from: {robot_usd_path}")
    except Exception as e:
        print(f"Could not load robot: {e}")
        print("Continuing with cubes only...")

    # Add dynamic cubes
    cube_positions = [
        (0.5, 0.5, 0.5),
        (0.5, -0.5, 0.5),
        (-0.5, 0.5, 0.5),
    ]

    for i, pos in enumerate(cube_positions):
        cube = world.scene.add(
            DynamicCuboid(
                prim_path=f"/World/Cube_{i}",
                name=f"cube_{i}",
                position=np.array(pos),
                scale=np.array([0.1, 0.1, 0.1]),
                color=np.array([1.0, 0.0, 0.0]),
            )
        )
        print(f"Added cube {i} at {pos}")

    return world


def main():
    """Main simulation loop."""

    print("Creating Isaac Sim scene...")
    world = create_isaac_scene()

    # Reset world to initialize physics
    print("Resetting world...")
    world.reset()

    # Run simulation loop
    print("Starting simulation loop (press ESC to exit)...")
    frame_count = 0

    while simulation_app.is_running():
        # Step physics and rendering
        world.step(render=True)

        frame_count += 1
        if frame_count % 100 == 0:
            print(f"Frame: {frame_count}")

    print("Simulation ended")
    simulation_app.close()


if __name__ == "__main__":
    main()
