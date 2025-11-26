# pick_up_example.py - Detailed Breakdown

This document provides a comprehensive breakdown of what `pick_up_example.py` does and how it works.

## Overview

`pick_up_example.py` is a complete demonstration of an autonomous pick-and-place task using the Denso Cobotta Pro 900 robot in NVIDIA Isaac Sim. The script sets up a simulation where the robot picks up a cube from one location and places it at another.

## File Structure

The script is organized into 7 main sections:

### 1. Initialize Isaac Sim Application (Lines 25-37)

```python
simulation_app = SimulationApp({"headless": False})
```

**What it does:**
- Creates the Isaac Sim application instance
- `headless=False` means the GUI viewer will be shown
- Must be created BEFORE importing other Isaac Sim modules

**Key Point:** All `isaacsim` imports must happen AFTER `SimulationApp` is created.

---

### 2. Create Simulation World (Lines 39-43)

```python
my_world = World(stage_units_in_meters=1.0)
```

**What it does:**
- Creates the simulation environment (world)
- Sets the unit system to meters
- The world manages:
  - Physics simulation
  - Scene rendering
  - Task execution
  - Time stepping

---

### 3. Define Pick-and-Place Task (Lines 45-60)

```python
target_position = np.array([-0.3, 0.6, 0])
target_position[2] = 0.0515 / 2.0
my_task = PickPlace(name="denso_pick_place", target_position=target_position)
```

**What it does:**
- **Defines target position:**
  - X: -0.3 meters (left of origin)
  - Y: 0.6 meters (forward from origin)
  - Z: 0.02575 meters (half cube height, so it sits on ground)

- **Creates PickPlace task:**
  - Loads Denso Cobotta robot with OnRobot RG6 gripper
  - Creates a cube object at default position (defined in pick_place.py)
  - Sets up target position for placing

**Default cube properties (from pick_place.py):**
- Size: 5.15 cm × 5.15 cm × 5.15 cm
- Initial position: Defined by parent class (typically around [0.3, 0.3, 0.0257])

---

### 4. Initialize Scene and Robot (Lines 62-73)

```python
my_world.add_task(my_task)
my_world.reset()
my_denso = my_world.scene.get_object("cobotta_robot")
```

**What it does:**
1. **Add task to world:**
   - Registers the task with the simulation
   - Calls `my_task.set_up_scene(scene)` which:
     - Loads robot USD file
     - Creates gripper
     - Adds cube to scene

2. **Reset world:**
   - Initializes physics
   - Prepares all objects
   - Sets objects to starting positions

3. **Get robot reference:**
   - Retrieves the robot object by name
   - Name "cobotta_robot" is set in `pick_place.py`

---

### 5. Initialize Controllers (Lines 75-100)

```python
my_controller = PickPlaceController(
    name="controller",
    robot_articulation=my_denso,
    gripper=my_denso.gripper
)
task_params = my_world.get_task("denso_pick_place").get_params()
articulation_controller = my_denso.get_articulation_controller()
```

**What it does:**

#### **PickPlaceController** (High-level)
Manages the pick-and-place state machine with these states:
1. **Move to pre-grasp** - Position above the object
2. **Approach** - Lower down to grasp height
3. **Grasp** - Close gripper
4. **Lift** - Raise object
5. **Move to target** - Navigate to placement position
6. **Lower** - Descend to placement height
7. **Release** - Open gripper
8. **Retreat** - Move away from placed object

#### **Task Parameters**
Retrieved from the task, includes:
- `cube_name`: Name of the object to pick ("target_cube")
- `robot_name`: Name of the robot ("cobotta_robot")
- `target_name`: Name of the target location

#### **ArticulationController** (Low-level)
- Directly controls robot joint positions/velocities
- Receives actions from PickPlaceController
- Sends commands to physics simulation

---

### 6. Main Simulation Loop (Lines 102-169)

```python
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        # Control logic here
```

**Flow for each frame:**

#### 6.1 Step Simulation
```python
my_world.step(render=True)
```
- Advances physics by one time step
- Updates object positions/velocities
- Renders the scene to the viewer

#### 6.2 Get Observations
```python
observations = my_world.get_observations()
```
Returns dictionary with:
```python
{
    "cobotta_robot": {
        "joint_positions": [j0, j1, j2, j3, j4, j5, g0, g1, ...],  # 12 joints total
    },
    "target_cube": {
        "position": [x, y, z],
        "orientation": [qx, qy, qz, qw],  # Quaternion
        "target_position": [tx, ty, tz],
    }
}
```

#### 6.3 Compute Actions
```python
actions = my_controller.forward(
    picking_position=observations["target_cube"]["position"],
    placing_position=observations["target_cube"]["target_position"],
    current_joint_positions=observations["cobotta_robot"]["joint_positions"],
    end_effector_offset=np.array([0, 0, 0.25]),
)
```

**Controller inputs:**
- `picking_position`: Where to pick up the cube
- `placing_position`: Where to place the cube
- `current_joint_positions`: Current robot configuration
- `end_effector_offset`: Offset from end-effector to grasp point (meters)

**Controller output (actions):**
- Joint position targets for the robot
- Gripper open/close commands
- These are passed to the articulation controller

#### 6.4 Apply Actions
```python
articulation_controller.apply_action(actions)
```
- Sends computed joint targets to the robot
- Physics simulation moves robot toward targets

#### 6.5 Check Completion
```python
if my_controller.is_done():
    print("done picking and placing")
```
- Controller signals when task is complete
- Robot has successfully placed the object

---

### 7. Cleanup (Lines 171-175)

```python
simulation_app.close()
```
- Closes the Isaac Sim application
- Shuts down physics simulation
- Releases resources

---

## Key Variables and Their Roles

| Variable | Type | Purpose |
|----------|------|---------|
| `simulation_app` | SimulationApp | Main Isaac Sim application instance |
| `my_world` | World | Simulation environment manager |
| `my_task` | PickPlace | Defines the pick-and-place task setup |
| `my_denso` | SingleManipulator | The Denso Cobotta robot object |
| `my_controller` | PickPlaceController | High-level task controller (state machine) |
| `articulation_controller` | ArticulationController | Low-level joint controller |
| `task_params` | dict | Task configuration (object/robot names) |
| `observations` | dict | Current state of world (robot + objects) |
| `actions` | ArticulationAction | Computed joint commands to apply |

---

## Data Flow Diagram

```
┌─────────────────┐
│  Observations   │ ← my_world.get_observations()
│ (Robot & Cube)  │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ PickPlace       │ ← my_controller.forward(...)
│ Controller      │
│ (State Machine) │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│    Actions      │
│ (Joint Targets) │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Articulation    │ ← articulation_controller.apply_action(...)
│ Controller      │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Physics Sim     │
│ (Robot Motion)  │
└─────────────────┘
```

---

## Coordinate System

Isaac Sim uses a **right-handed coordinate system**:
- **+X**: Right
- **+Y**: Forward
- **+Z**: Up

**Example positions:**
- `[0.3, 0.3, 0.03]` - Front-right, slightly above ground
- `[-0.3, 0.6, 0.03]` - Front-left, slightly above ground

**Units:** Meters

---

## Tunable Parameters

### Target Position (Lines 51-53)
```python
target_position = np.array([-0.3, 0.6, 0])
target_position[2] = 0.0515 / 2.0
```
**Modify to:** Change where the cube is placed

### End-Effector Offset (Line 155)
```python
end_effector_offset=np.array([0, 0, 0.25])
```
**Modify to:** Adjust grasp point relative to end-effector
- Increase Z: Grasp higher on object
- Decrease Z: Grasp lower on object
- X/Y: Lateral offset

### Cube Size (in pick_place.py line 27)
```python
cube_size=np.array([0.0515, 0.0515, 0.0515])
```
**Modify to:** Change object dimensions

### Cube Initial Position (in pick_place.py)
```python
cube_initial_position=np.array([0.3, 0.3, 0.03])
```
**Modify to:** Change where cube starts

---

## Common Modifications

### Change Pickup/Place Locations
```python
# In pick_up_example.py
target_position = np.array([0.5, 0.5, 0.03])  # New placement location

# In pick_place.py
cube_initial_position = np.array([0.2, 0.2, 0.03])  # New pickup location
```

### Adjust Gripper Behavior
```python
# In pick_place.py, modify gripper settings:
gripper = ParallelGripper(
    end_effector_prim_path="/World/cobotta/onrobot_rg6_base_link",
    joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],
    joint_opened_positions=np.array([0, 0]),        # Fully open
    joint_closed_positions=np.array([0.628, -0.628]),  # Fully closed
    action_deltas=np.array([-0.2, 0.2])            # Speed of opening/closing
)
```

### Change Object Size
```python
# In pick_place.py
cube_size=np.array([0.08, 0.06, 0.04])  # Different sized cuboid
```

---

## Troubleshooting

### Robot Not Moving
- Check that `my_world.is_playing()` is True
- Verify `my_controller` is properly initialized
- Ensure gripper is not None

### Object Not Picked Up
- Increase `end_effector_offset` Z value
- Check gripper is closing (watch joint values)
- Verify object is within robot's reach

### Simulation Crashes
- Make sure `SimulationApp` is created before other imports
- Check all file paths are correct
- Verify Isaac Sim installation

---

## Dependencies

**Required Files:**
- `pick_place.py` - Defines the PickPlace task
- `pick_place_controller.py` - Defines the PickPlaceController

**Required Assets:**
- Denso Cobotta Pro 900 USD file (loaded from Omniverse)
- OnRobot RG6 gripper (embedded in robot USD)

**Python Packages:**
- `isaacsim` (Isaac Sim Python API)
- `numpy`

---

## Execution Flow Summary

1. **Startup:** Create Isaac Sim app → Create world
2. **Setup:** Define task → Add to world → Reset → Get robot
3. **Control Loop:**
   - Step simulation
   - Get observations (robot state, object position)
   - Compute actions (controller state machine)
   - Apply actions (move robot)
   - Check if done
4. **Shutdown:** Close app

**Typical run time:** Until user closes window or task completes

---

## Related Files

- **pick_place.py** - Task definition (robot setup, cube creation)
- **pick_place_controller.py** - Controller implementation
- **follow_target_example.py** - Different example (no gripper)
- **custom_pick_up_example.py** - Custom object types
- **README_CUSTOM_OBJECTS.md** - Guide for custom objects

---

## Next Steps

After understanding this file:
1. Modify target positions to test different locations
2. Adjust gripper offset to optimize grasping
3. Change cube size to test with different objects
4. Explore custom object types (see custom_pick_up_example.py)
5. Implement your own tasks by extending PickPlace
