# Blueprints
Blueprints is a pythonic library that constructs Mujoco simulations and gives access to runtime data. It is designed to simplify access to mujoco and provide helpful subroutines for procedural generation, aiding environment randomization, curriculum learning, hindsight experiance replay and other RL paradigms aiming to manipulate the environment.

Once a certain structure of Mujoco elements has been build, it can be copied, many times throughout the world. The same objects through which the simulation has been constructed also serve as access objects to the simulations runtime data, no need of keeping track of indecies for ``mj_data.obj_type(index)`` calls.

Blueprints offers many conveniences for maipulating many objects attributes at once, implements standart agent interfaces for RL and bundles all sorts of data access in a finegrained indexing scheme through Views.

## Installation
Directly from PyPI:
```
$ pip install mujoco_blueprints
```

For RL integration (Gymnasium):
```
$ pip install mujoco_blueprints[rl]
```

For multi-agent RL (PettingZoo):
```
$ pip install mujoco_blueprints[multi-rl]
```

Alternatively from Github:
```
$ git clone https://github.com/mortimervonchappuis/mujoco_blueprints.git
```

## RL Integration

The `blueprints.rl` module provides base classes for creating Gymnasium and PettingZoo environments from any blueprints world. Define your world, pick your reward — the rest is handled automatically.

```python
from blueprints.rl import Env
import blueprints as blue

class MyEnv(Env):
    n_substeps = 4

    def setup(self):
        self.world = blue.World()
        agent = blue.Agent(name='robot', pos=[0, 0, 1])
        joint = blue.joints.Hinge(axis=[0, 0, 1])
        joint.attach(blue.actuators.Motor(gear=[100]),
                     blue.sensors.JointPos())
        agent.attach(blue.geoms.Sphere(size=0.1), joint)
        self.world.attach(agent)

    def reward(self, action):
        return self.agent.x_vel - 0.01 * sum(action**2)
```

The module also includes composable [reward functions](blueprints/rl/rewards.py), [observation/action wrappers](blueprints/rl/wrappers.py), and a [PettingZoo multi-agent wrapper](blueprints/rl/multi_env.py). See the [rl README](blueprints/rl/README.md) and [tutorials](tutorials/) for full details.

## Documentation

See [documentation](https://mujoco-blueprints.readthedocs.io/en/latest/index.html) for more details.