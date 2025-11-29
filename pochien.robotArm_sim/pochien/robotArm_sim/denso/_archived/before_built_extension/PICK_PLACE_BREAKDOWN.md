# pick_place.py - Detailed Breakdown

## Overview

`pick_place.py` defines the `PickPlace` task class that sets up the Denso Cobotta Pro 900 robot with an OnRobot RG6 gripper for pick-and-place operations in Isaac Sim.

This file is responsible for:
- Loading the robot USD model
- Configuring the gripper
- Setting default joint positions
- Creating the manipulator object

## File Structure

### Module-Level Docstring (Lines 1-26)

Provides overview of the module's purpose and usage examples.

**Key Information:**
- What the module does
- How to use it
- What components it provides

---

### Imports (Lines 28-33)

```python
from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.core.utils.stage import add_reference_to_stage
import isaacsim.core.api.tasks as tasks
from typing import Optional
import numpy as np
```

| Import | Purpose |
|--------|---------|
| `SingleManipulator` | Wrapper class for robot with gripper |
| `ParallelGripper` | Gripper controller for OnRobot RG6 |
| `add_reference_to_stage` | Loads USD files into scene |
| `tasks` | Base PickPlace task from Isaac Sim |
| `Optional` | Type hints for optional parameters |
| `numpy` | Array operations for positions |

---

### PickPlace Class (Lines 36-230)

#### Class Docstring (Lines 37-54)

Describes the class purpose, capabilities, and specifications.

**Robot Specifications:**
- **Model**: Denso Cobotta Pro 900
- **DOF**: 6 arm joints
- **Payload**: 900g
- **Reach**: 900mm

**Gripper Specifications:**
- **Model**: OnRobot RG6
- **Type**: 2-finger parallel
- **Total Joints**: 6 (2 controlled, 4 mimic)

**Default Cube:**
- **Size**: 5.15 cm × 5.15 cm × 5.15 cm

---

## Methods

### `__init__()` - Lines 55-105

#### Purpose
Initializes the pick-and-place task with configurable parameters.

#### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `name` | str | "denso_pick_place" | Task identifier |
| `cube_initial_position` | np.ndarray\|None | None | Starting [x,y,z] position |
| `cube_initial_orientation` | np.ndarray\|None | None | Starting quaternion [x,y,z,w] |
| `target_position` | np.ndarray\|None | None | Target [x,y,z] position |
| `offset` | np.ndarray\|None | None | Task offset |

#### What It Does

1. **Accepts user parameters**
2. **Calls parent class constructor** with:
   - Task name
   - Cube positions/orientation
   - **Fixed cube size**: [0.0515, 0.0515, 0.0515] meters
   - Task offset

#### Key Line

```python
cube_size=np.array([0.0515, 0.0515, 0.0515]),  # Line 102
```

**To change cube size**: Modify this line.

---

### `set_robot()` - Lines 107-230

#### Purpose
Loads and configures the Denso Cobotta robot with OnRobot RG6 gripper.

#### Returns
`SingleManipulator`: Configured robot object

#### Process Flow

```
1. Load Robot USD
   ↓
2. Configure Gripper
   ↓
3. Create Manipulator
   ↓
4. Set Default Joints
   ↓
5. Return Manipulator
```

---

### Section 1: Load Robot USD (Lines 169-173)

```python
asset_path = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots/Denso/cobotta_pro_900.usd"
add_reference_to_stage(usd_path=asset_path, prim_path="/World/cobotta")
```

**What it does:**
- Downloads Denso Cobotta USD from Omniverse
- Adds it to the scene at `/World/cobotta`

**USD Contents:**
- Robot geometry (meshes, links)
- Joint definitions (6 arm joints)
- Physics properties (masses, inertias)
- OnRobot RG6 gripper (embedded)

---

### Section 2: Configure Gripper (Lines 175-197)

```python
gripper = ParallelGripper(
    end_effector_prim_path="/World/cobotta/onrobot_rg6_base_link",
    joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],
    joint_opened_positions=np.array([0, 0]),
    joint_closed_positions=np.array([0.628, -0.628]),
    action_deltas=np.array([-0.2, 0.2])
)
```

#### Parameters Explained

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `end_effector_prim_path` | `/World/cobotta/onrobot_rg6_base_link` | Where gripper attaches to arm |
| `joint_prim_names` | `["finger_joint", "right_outer_knuckle_joint"]` | Two controlled joints |
| `joint_opened_positions` | `[0, 0]` | Fully open = 0 radians |
| `joint_closed_positions` | `[0.628, -0.628]` | Fully closed = ~36° each |
| `action_deltas` | `[-0.2, 0.2]` | Speed of opening/closing |

#### Gripper Joint Hierarchy

```
onrobot_rg6_base_link (end-effector)
├─ finger_joint (left finger) ← CONTROLLED
│  ├─ left_outer_knuckle_joint (mimic)
│  └─ left_inner_knuckle_joint (mimic)
└─ right_outer_knuckle_joint (right finger) ← CONTROLLED
   ├─ right_inner_knuckle_joint (mimic)
   └─ right_inner_finger_joint (mimic)
```

**Controlled joints**: 2 (left and right fingers)
**Mimic joints**: 4 (follow controlled joints automatically)

---

### Section 3: Create Manipulator (Lines 199-214)

```python
manipulator = SingleManipulator(
    prim_path="/World/cobotta",
    name="cobotta_robot",
    end_effector_prim_name="onrobot_rg6_base_link",
    gripper=gripper
)
```

#### Parameters Explained

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `prim_path` | `/World/cobotta` | Root of robot in scene |
| `name` | `cobotta_robot` | Name to reference robot |
| `end_effector_prim_name` | `onrobot_rg6_base_link` | Gripper mount point |
| `gripper` | `gripper` | Attach configured gripper |

**What SingleManipulator does:**
- Wraps robot USD as a controllable object
- Connects gripper to end-effector
- Provides methods like:
  - `get_joint_positions()`
  - `get_articulation_controller()`
  - `get_world_pose()`

---

### Section 4: Set Default Joints (Lines 216-228)

```python
joints_default_positions = np.zeros(12)
joints_default_positions[7] = 0.628
joints_default_positions[8] = 0.628
manipulator.set_joints_default_state(positions=joints_default_positions)
```

#### Joint Index Map (12 Total)

| Index | Joint Name | Type | Default Position |
|-------|------------|------|------------------|
| 0 | shoulder_pan_joint | Arm | 0.0 |
| 1 | shoulder_lift_joint | Arm | 0.0 |
| 2 | elbow_joint | Arm | 0.0 |
| 3 | wrist_1_joint | Arm | 0.0 |
| 4 | wrist_2_joint | Arm | 0.0 |
| 5 | wrist_3_joint | Arm | 0.0 |
| 6 | finger_joint | Gripper | 0.0 |
| 7 | left_outer_knuckle_joint | Gripper | **0.628** |
| 8 | left_inner_knuckle_joint | Gripper | **0.628** |
| 9 | right_outer_knuckle_joint | Gripper | 0.0 |
| 10 | right_inner_knuckle_joint | Gripper | 0.0 |
| 11 | right_inner_finger_joint | Gripper | 0.0 |

**Why set indices 7 and 8 to 0.628?**
- Keeps gripper **slightly open** at startup
- Prevents gripper from being fully closed initially
- 0.628 radians ≈ 36 degrees

---

## Scene Hierarchy

After `set_robot()` executes, the scene structure is:

```
/World
└─ /cobotta (robot root)
   ├─ base_link
   ├─ shoulder_link
   ├─ upper_arm_link
   ├─ forearm_link
   ├─ wrist_1_link
   ├─ wrist_2_link
   ├─ wrist_3_link
   └─ onrobot_rg6_base_link (end-effector)
      ├─ finger_joint
      │  ├─ left_outer_knuckle_joint
      │  └─ left_inner_knuckle_joint
      └─ right_outer_knuckle_joint
         ├─ right_inner_knuckle_joint
         └─ right_inner_finger_joint
```

---

## Coordinate System

**Robot base at origin**: [0, 0, 0]

**Typical working envelope:**
- **X**: -0.9m to +0.9m (left/right)
- **Y**: -0.9m to +0.9m (forward/back)
- **Z**: 0m to 1.8m (up from base)

**Gripper orientation:**
- Default: Points downward (-Z)
- Can rotate in all axes

---

## Customization Examples

### Change Cube Size

```python
# In __init__, line 102
cube_size=np.array([0.08, 0.06, 0.04]),  # Custom cuboid
```

### Change Gripper Closing Range

```python
# In set_robot(), lines 189-191
joint_opened_positions=np.array([0, 0]),           # Keep same
joint_closed_positions=np.array([0.8, -0.8]),      # Close more
action_deltas=np.array([-0.3, 0.3])                # Faster closing
```

### Change Default Arm Pose

```python
# In set_robot(), lines 220-225
joints_default_positions[0] = 0.5   # Rotate shoulder 0.5 rad
joints_default_positions[1] = -0.3  # Lower shoulder lift
# ... etc
```

---

## Common Issues

### Gripper Not Working

**Symptom**: Gripper doesn't open/close
**Causes**:
1. Joint names don't match USD
2. Gripper not initialized
3. Wrong closed/open positions

**Solution**:
- Check joint names in USD file
- Ensure gripper.initialize() is called
- Verify positions are within joint limits

### Robot Loads But Can't Move

**Symptom**: Robot appears but is frozen
**Causes**:
1. Default joint positions out of limits
2. Physics not enabled
3. Articulation controller not set up

**Solution**:
- Check joint limits in USD
- Verify world.reset() was called
- Ensure articulation_controller is created

### Cube Wrong Size

**Symptom**: Cube too big/small to grasp
**Solution**: Modify line 102 `cube_size` parameter

---

## Dependencies

**Required by this file:**
- Denso Cobotta Pro 900 USD (from Omniverse)
- Isaac Sim Python API
- NumPy

**Required by files using this:**
- `pick_up_example.py` (usage example)
- `pick_place_controller.py` (controller)

---

## Related Files

- **pick_up_example.py**: Uses this task class
- **pick_place_controller.py**: Controls the robot
- **follow_target.py**: Similar task without gripper
- **rmpflow.py**: RMPflow configuration

---

## Summary

`pick_place.py` is responsible for **scene setup**:
1. ✅ Loads robot USD model
2. ✅ Configures gripper
3. ✅ Sets default positions
4. ✅ Returns ready-to-use manipulator

**Not responsible for:**
- ❌ Motion control (that's the controller)
- ❌ Simulation loop (that's the example script)
- ❌ Observations (that's the world/task)

Think of it as the "**robot setup blueprint**" - it defines what the robot IS, not what it DOES.
