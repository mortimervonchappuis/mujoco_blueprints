# blueprints.rl

RL integration for mujoco_blueprints. Build Gymnasium and PettingZoo environments directly from blueprints worlds without boilerplate.

## Installation

```bash
pip install mujoco-blueprints[rl]          # Gymnasium support
pip install mujoco-blueprints[multi-rl]    # + PettingZoo support
```

## Quick Start

### Single-Agent Environment

Subclass `Env`, implement `setup()` and `reward()`:

```python
from blueprints.rl import Env
import blueprints as blue
import numpy as np

class CartPole(Env):
    n_substeps = 4

    def setup(self):
        self.world = blue.World(timestep=0.002)
        agent = blue.Agent(name='cart', pos=[0, 0, 1])

        # Cart: slides along x-axis
        cart_geom = blue.geoms.Box(x_length=0.4, y_length=0.2, z_length=0.1)
        cart_slide = blue.joints.Slide(axis=[1, 0, 0])
        cart_motor = blue.actuators.Motor(gear=[50])
        cart_sensor = blue.sensors.JointPos()
        cart_slide.attach(cart_motor, cart_sensor)
        agent.attach(cart_geom, cart_slide)

        # Pole: hinges from the cart
        pole_body = blue.Body(name='pole', z=0.05)
        pole_geom = blue.geoms.Capsule(radius=0.02, length=0.5, z=0.25)
        pole_hinge = blue.joints.Hinge(axis=[0, 1, 0])
        pole_hinge.attach(blue.sensors.JointPos(), blue.sensors.JointVel())
        pole_body.attach(pole_geom, pole_hinge)
        agent.attach(pole_body)

        self.world.attach(agent)

    def reward(self, action):
        return 1.0  # alive bonus

    def terminated(self):
        return abs(self.agent.z) < 0.3  # pole fell

# Use like any Gymnasium env
env = CartPole()
obs, info = env.reset()
obs, reward, terminated, truncated, info = env.step(env.action_space.sample())
env.close()
```

### Multi-Agent Environment

Subclass `MultiAgentEnv` for PettingZoo parallel environments:

```python
from blueprints.rl import MultiAgentEnv
import blueprints as blue

class Arena(MultiAgentEnv):
    def setup(self):
        self.world = blue.World()
        for i in range(3):
            agent = blue.Agent(name=f'robot_{i}', pos=[i * 2, 0, 1])
            joint = blue.joints.Hinge(axis=[0, 0, 1])
            joint.attach(blue.actuators.Motor(gear=[50]),
                         blue.sensors.JointPos())
            agent.attach(blue.geoms.Sphere(size=0.1), joint)
            self.world.attach(agent)

    def reward(self, agent_id, action):
        return float(self._agents_by_id[agent_id].x_vel)

env = Arena()
obs, infos = env.reset()
actions = {aid: env.action_space(aid).sample() for aid in env.agents}
obs, rewards, terms, truncs, infos = env.step(actions)
env.close()
```

## API Reference

### `Env` (Gymnasium)

Base class for single-agent environments.

**Required methods:**

| Method | Signature | Description |
|--------|-----------|-------------|
| `setup()` | `-> None` | Create `self.world` with at least one `blue.Agent` |
| `reward(action)` | `-> float` | Reward for the current state and action taken |

**Optional methods:**

| Method | Default | Description |
|--------|---------|-------------|
| `terminated()` | `False` | Episode terminal condition |
| `truncated()` | `False` | Episode truncation condition |
| `info()` | `{}` | Extra info dict for logging |
| `on_reset()` | no-op | Hook for procedural generation (called before `world.reset()`) |

**Class attributes:**

| Attribute | Default | Description |
|-----------|---------|-------------|
| `n_substeps` | `1` | MuJoCo steps per `env.step()` (frame skip) |
| `agent_name` | `None` | Agent name (auto-detects if only one agent) |
| `include_cameras` | `False` | Include camera images in observation space |
| `render_camera` | `None` | Camera name for `render()` |

### `MultiAgentEnv` (PettingZoo)

Base class for multi-agent parallel environments.

Same interface as `Env`, except `reward`, `terminated`, `truncated`, and `info` take an `agent_id` argument:

```python
def reward(self, agent_id, action) -> float: ...
def terminated(self, agent_id) -> bool: ...
```

Access agents by id via `self._agents_by_id[agent_id]`.

### `rewards` Module

Composable reward functions. Import and combine:

```python
from blueprints.rl import rewards

def reward(self, action):
    return (rewards.velocity(self.agent, axis='x')
          + rewards.alive_bonus(self.agent, min_z=0.2)
          - rewards.action_penalty(action, scale=0.01)
          - rewards.height_penalty(self.agent, target_z=1.0)
          + rewards.goal_distance(self.agent, goal=[10, 0])
          - rewards.contact_cost(self.agent))
```

| Function | Arguments | Description |
|----------|-----------|-------------|
| `velocity(agent, axis, scale)` | `axis='x'`, `scale=1.0` | Velocity along axis |
| `alive_bonus(agent, bonus, min_z, max_z)` | `bonus=1.0` | Constant bonus while in height bounds |
| `action_penalty(action, scale, norm)` | `scale=0.01`, `norm='l2'` | Penalize large actions |
| `height_penalty(agent, target_z, scale)` | `scale=1.0` | Penalize deviation from target height |
| `goal_distance(agent, goal, scale, axes)` | `axes='xy'` | Negative distance to goal position |
| `contact_cost(agent, scale)` | `scale=0.001` | Penalize contact forces (Touch sensors) |

### `wrappers` Module

Standard Gymnasium wrappers:

```python
from blueprints.rl.wrappers import NormalizeObservation, ActionClip, ActionScale

env = MyEnv()
env = ActionClip(env, low=-1.0, high=1.0)
env = NormalizeObservation(env)
```

| Wrapper | Description |
|---------|-------------|
| `FlattenObservation(env)` | Flatten Dict observation to single Box (excludes camera images) |
| `NormalizeObservation(env, epsilon)` | Running mean/std normalization |
| `ActionClip(env, low, high)` | Clip actions to a range |
| `ActionScale(env, low, high)` | Rescale actions from [-1, 1] to [low, high] |
