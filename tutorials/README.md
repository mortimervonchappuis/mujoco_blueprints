# Tutorials

Hands-on examples for building and training RL environments with blueprints.

## Double Pendulum Swing-Up

**File:** [`double_pendulum.py`](double_pendulum.py)

Builds a double pendulum (acrobot) and trains a PPO agent to swing it upright using Stable-Baselines3.

```bash
pip install mujoco-blueprints[rl] stable-baselines3
python tutorials/double_pendulum.py
```

**What it covers:**
- Building a multi-body kinematic chain (two links with hinge joints)
- Attaching sensors (JointPos, JointVel) and actuators (Motor)
- Subclassing `blueprints.rl.Env`
- Using `blueprints.rl.rewards` for reward composition
- Using `blueprints.rl.wrappers` (ActionClip, NormalizeObservation)
- Training with Stable-Baselines3 PPO
- Evaluating a trained policy
