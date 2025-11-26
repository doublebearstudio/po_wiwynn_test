# Guide: Creating Custom Pickup Objects in Isaac Sim

This guide explains how to define your own pickup objects instead of using the default cube.

## Important Note

The Isaac Sim `tasks.PickPlace` base class has complex internal setup requirements. **Overriding `set_up_scene()` is not recommended** as it can break the task's internal state management.

## Recommended Approaches

### Approach 1: Modify Cube Size (Simplest)

The easiest way to customize the object is to change the cube size in the `__init__` method:

```python
class PickPlace(tasks.PickPlace):
    def __init__(self, name="denso_pick_place", **kwargs):
        tasks.PickPlace.__init__(
            self,
            name=name,
            cube_size=np.array([0.1, 0.05, 0.03]),  # Custom size [length, width, height]
            **kwargs
        )
```

### Approach 2: Create Custom Task Class (Advanced)

Instead of inheriting from `tasks.PickPlace`, create your own task that doesn't rely on the base class's object creation:

```python
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.objects import DynamicCylinder

class CustomPickPlace(BaseTask):
    def __init__(self, name="custom_pick_place", object_type="cylinder"):
        super().__init__(name=name)
        self.object_type = object_type

    def set_up_scene(self, scene):
        # Set up your robot
        robot = self.set_robot()
        scene.add(robot)

        # Create your custom object
        if self.object_type == "cylinder":
            pickup_object = DynamicCylinder(
                prim_path="/World/pickup_cylinder",
                name="pickup_object",
                position=np.array([0.3, 0.3, 0.05]),
                radius=0.025,
                height=0.05,
                color=np.array([0.0, 0.5, 1.0])
            )

        scene.add(pickup_object)

    def get_observations(self):
        # Implement observation logic
        pass

    def get_params(self):
        # Return task parameters
        pass
```

**Note**: This approach requires implementing all task logic yourself (observations, parameters, etc.)

### Approach 3: Manually Add Objects After Scene Setup

Create the default cube, then add additional objects after `my_world.reset()`:

```python
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from isaacsim.core.api import World
from isaacsim.core.objects import DynamicCylinder
import numpy as np
from pick_place import PickPlace

my_world = World(stage_units_in_meters=1.0)

# Set up the default pick and place task
target_position = np.array([-0.3, 0.6, 0])
target_position[2] = 0.0515 / 2.0
my_task = PickPlace(name="denso_pick_place", target_position=target_position)

my_world.add_task(my_task)
my_world.reset()

# Now add custom objects to the scene AFTER reset
custom_cylinder = DynamicCylinder(
    prim_path="/World/custom_cylinder",
    name="custom_cylinder",
    position=np.array([0.5, 0.5, 0.05]),
    radius=0.03,
    height=0.08,
    color=np.array([1.0, 0.0, 0.0])
)
my_world.scene.add(custom_cylinder)

# Continue with the rest of your code...
```

### Approach 4: Modify Existing Cube After Creation

Access and modify the cube after it's created by the base class:

```python
my_world.add_task(my_task)
my_world.reset()

# Get the cube object created by the task
cube = my_world.scene.get_object("target_cube")

# Modify its properties
if cube:
    # Change color
    cube.set_default_color(np.array([1.0, 0.0, 0.0]))  # Red

    # Change scale (effectively changing size)
    cube.set_local_scale(np.array([2.0, 2.0, 2.0]))  # 2x larger
```

## Available Geometry Types (For Custom Approaches)

When creating custom objects, you can use:

- **DynamicCylinder** - Physics-enabled cylinder
- **DynamicSphere** - Physics-enabled sphere
- **DynamicCone** - Physics-enabled cone
- **DynamicCuboid** - Physics-enabled box
- **VisualCylinder** - Static visual-only cylinder
- **VisualSphere** - Static visual-only sphere
- **VisualCone** - Static visual-only cone
- **VisualCuboid** - Static visual-only box

## Loading USD Files

You can load custom USD models (mugs, tools, CAD parts):

```python
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.prims import RigidPrim

# Load USD file
usd_path = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.0/Isaac/Props/Mug/mug.usd"
add_reference_to_stage(usd_path=usd_path, prim_path="/World/my_mug")

# Wrap it as a RigidPrim for physics
mug = RigidPrim(
    prim_path="/World/my_mug",
    name="mug",
    position=np.array([0.3, 0.3, 0.1])
)
my_world.scene.add(mug)
```

### Isaac Sim Built-in Props

Browse available props at:
- `omniverse://localhost/NVIDIA/Assets/Isaac/4.0/Isaac/Props/`
- Or online: https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.0/Isaac/Props/

Examples:
- Mug: `.../Isaac/Props/Mug/mug.usd`
- Block: `.../Isaac/Props/Blocks/block_instanceable.usd`
- Bin: `.../Isaac/Props/KLT_Bin/small_KLT.usd`

## Summary

1. **Simplest**: Modify `cube_size` parameter in `__init__`
2. **Easy**: Add custom objects manually after `my_world.reset()`
3. **Moderate**: Modify the created cube's properties after setup
4. **Advanced**: Create a fully custom task class inheriting from `BaseTask`

**Avoid**: Overriding `set_up_scene()` in `tasks.PickPlace` subclasses unless you fully replicate all parent class functionality.

## Current Code Status

The current `denso/pick_place.py` uses the **default cube** from the parent class. It's the most stable approach and works reliably with the pick-and-place controller.

To customize:
- Change `cube_size` in line 27 to modify dimensions
- Add custom objects in `pick_up_example.py` after line 18 (`my_world.reset()`)
