#!/usr/bin/env python3
"""
Isaac Gym PPO Training - CartPole Example
Demonstrates vectorized environments and basic RL training loop.

Requirements:
    - Isaac Gym Preview 4
    - PyTorch 2.0+
"""

import torch
import torch.nn as nn
import numpy as np
from isaacgym import gymapi, gymtorch


class ActorCritic(nn.Module):
    """Simple actor-critic network for CartPole."""

    def __init__(self, obs_dim=4, act_dim=1, hidden_dim=64):
        super().__init__()

        # Shared feature extractor
        self.features = nn.Sequential(
            nn.Linear(obs_dim, hidden_dim),
            nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU()
        )

        # Actor head (policy)
        self.actor = nn.Linear(hidden_dim, act_dim)

        # Critic head (value function)
        self.critic = nn.Linear(hidden_dim, 1)

    def forward(self, obs):
        features = self.features(obs)
        action_logits = self.actor(features)
        value = self.critic(features)
        return action_logits, value


def create_cartpole_envs(gym, sim, num_envs=512):
    """Create vectorized CartPole environments."""

    # Define environment grid
    num_per_row = int(np.sqrt(num_envs))
    spacing = 2.0
    lower = gymapi.Vec3(-spacing, 0.0, -spacing)
    upper = gymapi.Vec3(spacing, spacing, spacing)

    # Load CartPole asset
    asset_root = "assets"
    asset_file = "cartpole.urdf"

    asset_options = gymapi.AssetOptions()
    asset_options.fix_base_link = True

    cartpole_asset = gym.load_asset(sim, asset_root, asset_file, asset_options)

    # Create environments
    envs = []
    actor_handles = []

    for i in range(num_envs):
        env = gym.create_env(sim, lower, upper, num_per_row)
        envs.append(env)

        pose = gymapi.Transform()
        pose.p = gymapi.Vec3(0.0, 0.0, 0.0)

        actor_handle = gym.create_actor(env, cartpole_asset, pose, f"cartpole_{i}", i, 1)
        actor_handles.append(actor_handle)

    return envs, actor_handles


def compute_rewards(pole_angles, cart_positions):
    """Compute rewards for CartPole task."""

    # Reward for keeping pole upright
    angle_reward = 1.0 - torch.abs(pole_angles) / (np.pi / 2)

    # Penalty for cart position
    position_penalty = -0.1 * torch.abs(cart_positions)

    # Total reward
    rewards = angle_reward + position_penalty

    return rewards


def main():
    """Main PPO training loop."""

    # Initialize Isaac Gym
    gym = gymapi.acquire_gym()

    # Create simulation
    sim_params = gymapi.SimParams()
    sim_params.dt = 1.0 / 60.0
    sim_params.substeps = 2
    sim_params.up_axis = gymapi.UP_AXIS_Z
    sim_params.gravity = gymapi.Vec3(0.0, 0.0, -9.81)

    sim = gym.create_sim(0, 0, gymapi.SIM_PHYSX, sim_params)

    # Create environments
    num_envs = 512
    print(f"Creating {num_envs} parallel CartPole environments...")
    envs, actor_handles = create_cartpole_envs(gym, sim, num_envs)

    # Initialize policy
    device = torch.device("cuda:0")
    policy = ActorCritic(obs_dim=4, act_dim=1, hidden_dim=64).to(device)
    optimizer = torch.optim.Adam(policy.parameters(), lr=3e-4)

    # Training loop
    num_iterations = 1000
    print(f"\nTraining for {num_iterations} iterations...")

    for iteration in range(num_iterations):
        # Step simulation
        gym.simulate(sim)
        gym.fetch_results(sim, True)

        # Get observations (simplified - normally from state tensors)
        obs = torch.randn(num_envs, 4, device=device)  # Placeholder

        # Forward pass
        action_logits, values = policy(obs)
        actions = torch.tanh(action_logits)

        # Apply actions (simplified)
        # In real implementation, write to force/torque tensors

        # Compute rewards
        pole_angles = obs[:, 2]  # Example state indexing
        cart_positions = obs[:, 0]
        rewards = compute_rewards(pole_angles, cart_positions)

        # PPO update (simplified)
        advantages = rewards - values.squeeze()
        actor_loss = -(advantages.detach() * action_logits).mean()
        critic_loss = advantages.pow(2).mean()
        loss = actor_loss + 0.5 * critic_loss

        optimizer.zero_grad()
        loss.backward()
        optimizer.step()

        # Logging
        if iteration % 100 == 0:
            mean_reward = rewards.mean().item()
            print(f"Iteration {iteration}: Mean Reward = {mean_reward:.3f}, Loss = {loss.item():.4f}")

    print("\nTraining complete!")
    print(f"Final mean reward: {rewards.mean().item():.3f}")

    # Cleanup
    gym.destroy_sim(sim)


if __name__ == "__main__":
    main()
