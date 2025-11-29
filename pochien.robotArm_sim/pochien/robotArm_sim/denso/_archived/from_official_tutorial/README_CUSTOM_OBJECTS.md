# Custom Pick and Place with Different Object Types

This directory contains a custom pick-and-place implementation that allows you to pick up different object types (cylinders, spheres, cubes, or USD models).

## Files

- **`custom_pick_place.py`**: Custom task class that inherits from `BaseTask` with full control over object creation
- **`custom_pick_up_example.py`**: Example script demonstrating how to use the custom task
- **`pick_place.py`**: Original task using Isaac Sim's default `tasks.PickPlace` (uses cube only)
- **`pick_up_example.py`**: Example using the original task

## Quick Start

### Run with Different Object Types

```bash
# Run with cylinder (default)
python custom_pick_up_example.py
```

To try different objects, edit line 19 in `custom_pick_up_example.py`:

```python
object_type="cylinder",  # Options: "cylinder", "sphere", "cube", "usd"
```

## Object Types

### 1. Cylinder
```python
my_task = CustomPickPlace(
    name="custom_pick_place",
    object_type="cylinder",
    object_initial_position=np.array([0.3, 0.3, 0.05]),
    target_position=np.array([-0.3, 0.6, 0.05])
)
```
- **Radius**: 2.5 cm
- **Height**: 5 cm
- **Color**: Blue

### 2. Sphere
```python
my_task = CustomPickPlace(
    name="custom_pick_place",
    object_type="sphere",
    object_initial_position=np.array([0.3, 0.3, 0.05]),
    target_position=np.array([-0.3, 0.6, 0.05])
)
```
- **Radius**: 3 cm
- **Color**: Orange

### 3. Cube
```python
my_task = CustomPickPlace(
    name="custom_pick_place",
    object_type="cube",
    object_initial_position=np.array([0.3, 0.3, 0.05]),
    target_position=np.array([-0.3, 0.6, 0.05])
)
```
- **Size**: 5.15 cm × 5.15 cm × 5.15 cm
- **Color**: Red

### 4. USD Model (e.g., Mug)
```python
my_task = CustomPickPlace(
    name="custom_pick_place",
    object_type="usd",
    object_initial_position=np.array([0.3, 0.3, 0.1]),
    target_position=np.array([-0.3, 0.6, 0.1])
)
```
- **Default**: Loads a mug from Isaac Sim assets
- **Customize**: Edit `_create_usd_object()` in `custom_pick_place.py` to load your own USD file

## Customizing Object Properties

### Change Object Size/Dimensions

Edit the corresponding method in `custom_pick_place.py`:

**For Cylinder** (lines 99-106):
```python
def _create_cylinder(self) -> DynamicCylinder:
    return DynamicCylinder(
        prim_path=f"/World/{self._object_name}",
        name=self._object_name,
        position=self._object_initial_position,
        radius=0.03,   # Change radius here (in meters)
        height=0.08,   # Change height here (in meters)
        color=np.array([0.0, 0.5, 1.0])
    )
```

**For Sphere** (lines 108-115):
```python
def _create_sphere(self) -> DynamicSphere:
    return DynamicSphere(
        prim_path=f"/World/{self._object_name}",
        name=self._object_name,
        position=self._object_initial_position,
        radius=0.04,  # Change radius here (in meters)
        color=np.array([1.0, 0.5, 0.0])
    )
```

**For Cube** (lines 117-124):
```python
def _create_cube(self) -> DynamicCuboid:
    return DynamicCuboid(
        prim_path=f"/World/{self._object_name}",
        name=self._object_name,
        position=self._object_initial_position,
        size=np.array([0.08, 0.06, 0.04]),  # [length, width, height] in meters
        color=np.array([1.0, 0.0, 0.0])
    )
```

### Change Object Colors

Colors are RGB values from 0-1:
- **Red**: `np.array([1.0, 0.0, 0.0])`
- **Green**: `np.array([0.0, 1.0, 0.0])`
- **Blue**: `np.array([0.0, 0.0, 1.0])`
- **Yellow**: `np.array([1.0, 1.0, 0.0])`
- **Purple**: `np.array([0.5, 0.0, 0.5])`
- **Orange**: `np.array([1.0, 0.5, 0.0])`

### Load Your Own USD Model

Edit `_create_usd_object()` in `custom_pick_place.py` (lines 126-134):

```python
def _create_usd_object(self) -> RigidPrim:
    # Replace with your USD file path
    usd_path = "D:/my_models/my_part.usd"  # Local file
    # Or use Omniverse: "omniverse://localhost/Projects/my_part.usd"
    # Or use Isaac Sim assets: "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.0/Isaac/Props/..."

    add_reference_to_stage(usd_path=usd_path, prim_path=f"/World/{self._object_name}")

    return RigidPrim(
        prim_path=f"/World/{self._object_name}",
        name=self._object_name,
        position=self._object_initial_position
    )
```

## Adding New Object Types

You can add new object types by:

1. **Add a new creation method** in `custom_pick_place.py`:
```python
def _create_cone(self) -> DynamicCone:
    from isaacsim.core.objects import DynamicCone
    return DynamicCone(
        prim_path=f"/World/{self._object_name}",
        name=self._object_name,
        position=self._object_initial_position,
        radius=0.03,
        height=0.06,
        color=np.array([1.0, 0.0, 1.0])  # Magenta
    )
```

2. **Add the case in `set_up_scene()`** (around line 73):
```python
elif self.object_type == "cone":
    pickup_object = self._create_cone()
```

3. **Use it** in your example:
```python
my_task = CustomPickPlace(object_type="cone", ...)
```

## Tuning the Gripper

Different objects may require different gripper offsets. Adjust in `custom_pick_up_example.py` line 48:

```python
end_effector_offset=np.array([0, 0, 0.25]),  # [x, y, z] offset in meters
```

**Guidelines**:
- **Small objects**: Reduce Z offset (e.g., `0.20`)
- **Tall objects**: Increase Z offset (e.g., `0.30`)
- **X/Y offset**: Usually keep at 0 unless object is off-center

## Comparison: Custom vs Original Task

| Feature | `custom_pick_place.py` | `pick_place.py` |
|---------|------------------------|-----------------|
| Object types | Cylinder, Sphere, Cube, USD | Cube only |
| Customization | Full control | Limited |
| Base class | `BaseTask` | `tasks.PickPlace` |
| Complexity | More code to maintain | Simpler |
| Flexibility | High | Low |

## Troubleshooting

### Object Not Being Picked Up
1. Check object position is reachable by robot
2. Adjust `end_effector_offset`
3. Verify gripper can physically grasp the object (check size)

### Gripper Not Closing
1. Check gripper initialization in `pick_place_controller.py`
2. Verify gripper joint positions in `set_robot()` method

### Object Falls Through Floor
1. Ensure Z position is at least half the object's height
2. For cylinders: Z ≥ height/2
3. For spheres: Z ≥ radius

### Scene Loads but Script Crashes
1. Make sure all imports are available
2. Check Isaac Sim version compatibility
3. Verify USD file paths are accessible

## Examples

### Example 1: Pick Up Small Red Cube
```python
my_task = CustomPickPlace(
    name="pick_red_cube",
    object_type="cube",
    object_initial_position=np.array([0.4, 0.2, 0.025]),
    target_position=np.array([-0.4, 0.2, 0.025])
)
```

### Example 2: Pick Up Blue Cylinder
```python
my_task = CustomPickPlace(
    name="pick_cylinder",
    object_type="cylinder",
    object_initial_position=np.array([0.3, 0.3, 0.05]),
    target_position=np.array([0.3, -0.3, 0.05])
)
```

### Example 3: Pick Up Orange Sphere
```python
my_task = CustomPickPlace(
    name="pick_sphere",
    object_type="sphere",
    object_initial_position=np.array([0.35, 0.35, 0.03]),
    target_position=np.array([-0.35, -0.35, 0.03])
)
```

## Next Steps

1. Try each object type to see which works best
2. Experiment with different sizes and colors
3. Load your own CAD models as USD files
4. Create multi-object scenarios by extending the task
5. Add collision detection or object classification

For more details, see the main guide: `docs/CUSTOM_OBJECTS_GUIDE.md`
