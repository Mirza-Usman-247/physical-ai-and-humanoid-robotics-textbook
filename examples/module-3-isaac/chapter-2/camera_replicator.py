#!/usr/bin/env python3
"""
Isaac Sim Replicator - Synthetic Data Generation
Demonstrates camera setup, domain randomization, and annotation capture.

Requirements:
    - Isaac Sim 4.0+ installed
    - NVIDIA GPU with RTX support
"""

from isaacsim import SimulationApp

simulation_app = SimulationApp({"headless": False})

import omni.replicator.core as rep
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np


def setup_replicator_scene():
    """Create scene with randomization for synthetic data generation."""

    # Initialize World
    world = World()
    world.scene.add_default_ground_plane()

    # Add objects to randomize
    assets_root = get_assets_root_path()
    if assets_root:
        # Add various objects for detection training
        object_usd = assets_root + "/Isaac/Props/Blocks/block_instanceable.usd"

        for i in range(5):
            add_reference_to_stage(
                usd_path=object_usd,
                prim_path=f"/World/Block_{i}"
            )

    # Create camera with Replicator
    camera = rep.create.camera(
        position=(2.0, 2.0, 1.5),
        look_at=(0, 0, 0.5)
    )

    # Define randomization
    with rep.trigger.on_frame(num_frames=100):
        # Randomize object positions
        with rep.create.group([f"/World/Block_{i}" for i in range(5)]):
            rep.modify.pose(
                position=rep.distribution.uniform((-1, -1, 0.5), (1, 1, 1.5)),
                rotation=rep.distribution.uniform((0, 0, 0), (360, 360, 360))
            )

        # Randomize lighting
        rep.randomizer.light(
            intensity=rep.distribution.uniform(800, 1500),
            temperature=rep.distribution.uniform(4000, 7000)
        )

        # Randomize materials
        rep.randomizer.color(
            textures=rep.distribution.choice([
                "/Materials/Red",
                "/Materials/Blue",
                "/Materials/Green"
            ])
        )

    # Setup output writer
    rp = rep.WriterRegistry.get("BasicWriter")
    rp.initialize(
        output_dir="output/synthetic_data",
        rgb=True,
        semantic_segmentation=True,
        instance_segmentation=True,
        distance_to_camera=True,
        bounding_box_2d_tight=True
    )
    rp.attach([camera])

    return world


def main():
    """Run synthetic data generation."""

    print("Setting up Replicator scene...")
    world = setup_replicator_scene()

    print("Resetting world...")
    world.reset()

    print("Generating synthetic data (100 frames)...")
    print("Output will be saved to: output/synthetic_data/")

    # Run orchestration
    rep.orchestrator.run()

    # Step simulation
    frame_count = 0
    while simulation_app.is_running() and frame_count < 100:
        world.step(render=True)
        frame_count += 1

        if frame_count % 10 == 0:
            print(f"Generated {frame_count}/100 frames")

    print("\nData generation complete!")
    print("Check output/synthetic_data/ for:")
    print("  - RGB images")
    print("  - Semantic segmentation masks")
    print("  - Instance segmentation masks")
    print("  - Bounding box annotations")

    simulation_app.close()


if __name__ == "__main__":
    main()
