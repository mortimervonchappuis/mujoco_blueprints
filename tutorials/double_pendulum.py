"""
Double Pendulum Swing-Up with PPO
==================================

This tutorial builds a double pendulum (acrobot) environment using
blueprints.rl and trains a PPO agent to swing it upright using
Stable-Baselines3.

Requirements::

    pip install mujoco-blueprints[rl] stable-baselines3

The pendulum starts hanging downward. The agent controls a motor on the
first joint and must learn to swing both links upright (tips as high as
possible).

Run this script to train for 100k steps, then watch the result:

    python tutorials/double_pendulum.py
"""

import numpy as np
import blueprints as blue
from blueprints.rl import Env, rewards
from blueprints.rl.wrappers import NormalizeObservation, ActionClip


# ── Environment ──────────────────────────────────────────────────────────

class DoublePendulum(Env):
    """
    A double pendulum (acrobot) swing-up task.

    The pendulum is anchored at a fixed point. Two links hang downward.
    A motor on the first joint applies torque. The goal is to swing the
    tip of the second link as high as possible.

    Observations (6-dim):
        joint_0 position, joint_1 position,
        joint_0 velocity, joint_1 velocity,
        tip x position,   tip z position

    Action (1-dim):
        torque on the first joint, clipped to [-1, 1]
    """
    n_substeps = 4

    def setup(self):
        self.world = blue.World(timestep=0.002)

        # ── Agent (fixed anchor point) ──
        agent = blue.Agent(name='pendulum', pos=[0, 0, 2])

        # ── First link ──
        link1 = blue.Body(name='link1')
        link1_geom = blue.geoms.Capsule(
            radius=0.04,
            length=0.5,
            z=-0.25,
            color='#2196F3',
        )
        link1_joint = blue.joints.Hinge(
            name='joint1',
            axis=[0, 1, 0],
            damping=0.05,
        )
        link1_motor = blue.actuators.Motor(
            name='motor1',
            gear=[2],
            ctrlrange=[-1, 1],
        )
        link1_joint.attach(
            link1_motor,
            blue.sensors.JointPos(name='j1_pos'),
            blue.sensors.JointVel(name='j1_vel'),
        )
        link1.attach(link1_geom, link1_joint)

        # ── Second link ──
        link2 = blue.Body(name='link2', z=-0.5)
        link2_geom = blue.geoms.Capsule(
            radius=0.03,
            length=0.5,
            z=-0.25,
            color='#FF9800',
        )
        link2_joint = blue.joints.Hinge(
            name='joint2',
            axis=[0, 1, 0],
            damping=0.05,
        )
        link2_joint.attach(
            blue.sensors.JointPos(name='j2_pos'),
            blue.sensors.JointVel(name='j2_vel'),
        )
        link2.attach(link2_geom, link2_joint)

        # ── Tip site (for measuring height) ──
        tip = blue.sites.Sphere(name='tip', size=0.05, z=-0.5, color='red')
        link2.attach(tip)

        # ── Assemble ──
        link1.attach(link2)
        agent.attach(link1)
        self.world.attach(agent)

        # ── Visual extras ──
        self.world.attach(blue.Light(z=5))

    def reward(self, action):
        # Reward = tip height (higher is better) minus action cost.
        # The anchor is at z=2. Fully upright tip reaches z=2+1.0=3.0.
        # Hanging down, tip is at z=2-1.0=1.0.
        tip_z = self._tip_z()
        return tip_z - 2.0 - rewards.action_penalty(action, scale=0.1)

    def terminated(self):
        return False

    def truncated(self):
        return False

    def info(self):
        return {'tip_z': self._tip_z()}

    def _tip_z(self):
        """Get the tip site's global z position."""
        import mujoco
        site_id = mujoco.mj_name2id(self.world.model, mujoco.mjtObj.mjOBJ_SITE, 'tip')
        return float(self.world.data.site_xpos[site_id][2])


# ── Training ─────────────────────────────────────────────────────────────

def train(total_timesteps=100_000):
    """Train a PPO agent on the double pendulum swing-up task."""
    try:
        from stable_baselines3 import PPO
    except ImportError:
        print("stable-baselines3 is required for training.")
        print("Install with: pip install stable-baselines3")
        return None, None

    # Create and wrap the environment
    env = DoublePendulum()
    env = ActionClip(env, low=-1.0, high=1.0)
    env = NormalizeObservation(env)

    print(f"Observation space: {env.observation_space}")
    print(f"Action space:      {env.action_space}")
    print(f"Training for {total_timesteps} steps...")

    model = PPO(
        "MlpPolicy",
        env,
        learning_rate=3e-4,
        n_steps=2048,
        batch_size=64,
        n_epochs=10,
        gamma=0.99,
        verbose=1,
    )
    model.learn(total_timesteps=total_timesteps)
    env.close()
    return model, DoublePendulum


# ── Evaluation ───────────────────────────────────────────────────────────

def evaluate(model, env_class, episodes=5, steps_per_episode=500):
    """Run the trained policy and print results."""
    env = env_class()
    env = ActionClip(env, low=-1.0, high=1.0)

    for ep in range(episodes):
        obs, info = env.reset()
        total_reward = 0.0
        max_tip_z = -float('inf')
        for _ in range(steps_per_episode):
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, info = env.step(action)
            total_reward += reward
            max_tip_z = max(max_tip_z, info['tip_z'])
            if terminated or truncated:
                break
        print(f"Episode {ep+1}: reward={total_reward:.1f}, max_tip_z={max_tip_z:.3f}")
    env.close()


# ── Main ─────────────────────────────────────────────────────────────────

if __name__ == '__main__':
    model, env_class = train(total_timesteps=100_000)
    if model is not None:
        print("\nEvaluation:")
        evaluate(model, env_class)
