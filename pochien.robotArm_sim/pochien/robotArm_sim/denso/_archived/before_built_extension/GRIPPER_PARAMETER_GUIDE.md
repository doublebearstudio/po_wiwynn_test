# Gripper Parameter Configuration Guide

## How to Find `joint_closed_positions` and `action_deltas` for ParallelGripper

This guide explains how to determine the correct values for gripper configuration parameters in pick_place.py.

---

## 1. Understanding `joint_closed_positions`

### What It Is
The joint angles (in radians) when the gripper fingers are fully closed.

### Current Values
```python
joint_closed_positions=np.array([0.628, -0.628])
```
- Left finger: 0.628 radians ≈ 36 degrees
- Right finger: -0.628 radians ≈ -36 degrees (opposite direction)

---

## Method 1: Manual Inspection in Isaac Sim (Recommended)

### Step-by-Step Process

1. **Load the Robot in Isaac Sim**
   ```python
   from isaacsim import SimulationApp
   simulation_app = SimulationApp({"headless": False})

   from omni.isaac.core.utils.stage import add_reference_to_stage
   asset_path = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots/Denso/cobotta_pro_900.usd"
   add_reference_to_stage(usd_path=asset_path, prim_path="/World/cobotta")
   ```

2. **Open the Property Panel**
   - Window → Property
   - Select the robot in the stage tree

3. **Find Gripper Joints**
   - Navigate to: `/World/cobotta/onrobot_rg6_base_link`
   - Look for joints:
     - `finger_joint` (left finger)
     - `right_outer_knuckle_joint` (right finger)

4. **Manually Close the Gripper**
   - Click on `finger_joint`
   - In the Property panel, find the joint drive or articulation section
   - Look for "Position" or "State: Position" field
   - Manually adjust the value and observe the gripper closing
   - Note the value when fingers touch each other (fully closed)

5. **Repeat for Right Finger**
   - Click on `right_outer_knuckle_joint`
   - Adjust position until it's fully closed
   - Note the value (will be opposite sign)

6. **Verify the Range**
   - Joint limits are usually defined in the USD file
   - Check "Lower Limit" and "Upper Limit" properties
   - Closed position should be within these limits

### Example Output
```
finger_joint (left):
  - Fully open: 0.0 rad
  - Fully closed: 0.628 rad (36°)
  - Joint limit: 0 to 0.7 rad

right_outer_knuckle_joint (right):
  - Fully open: 0.0 rad
  - Fully closed: -0.628 rad (-36°)
  - Joint limit: -0.7 to 0 rad
```

---

## Method 2: Calculate from Gripper Geometry

### OnRobot RG6 Specifications

The OnRobot RG6 gripper has:
- **Stroke**: 160mm (0-160mm adjustable)
- **Finger length**: Depends on configuration
- **Parallel jaw mechanism**: Both fingers move symmetrically

### Calculation Steps

1. **Measure Finger Geometry**
   - When fully open: fingers are 160mm apart (at widest)
   - When fully closed: fingers touch (0mm gap)

2. **Determine Finger Mechanism**
   - Parallel grippers use linkage mechanism
   - Joint angle relates to linear finger displacement
   - For RG6: approximately 36° gives full stroke

3. **Relationship to Cube Size**

   **IMPORTANT**: `joint_closed_positions` is **NOT directly related to cube_size**

   - `joint_closed_positions`: Maximum closure angle (gripper fully closed)
   - `cube_size`: Size of object to grasp
   - The gripper will stop closing when it touches the object, regardless of max angle

   **Why 0.628 rad works for 5.15cm cube**:
   - Gripper fully closed gap: ~0mm
   - Gripper at 0.628 rad: fingers can grip objects from 0mm to 160mm
   - 5.15cm cube: well within gripper range
   - Controller automatically stops at contact

4. **Formula (Approximate)**

   For parallel jaw grippers:
   ```
   joint_angle = arctan(finger_displacement / lever_arm_length)
   ```

   For RG6:
   - Full stroke (160mm) ≈ 0.628 rad (36°)
   - This is empirical from the gripper design

---

## Method 3: Read from USD File

### Using USD Inspection

1. **Download the Robot USD**
   ```bash
   # The robot is hosted at:
   https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots/Denso/cobotta_pro_900.usd
   ```

2. **Open in USD View or Text Editor**
   ```bash
   usdview cobotta_pro_900.usd
   # OR
   usdedit cobotta_pro_900.usd
   ```

3. **Search for Joint Definitions**
   Look for joint definitions like:
   ```usd
   def Joint "finger_joint"
   {
       float physics:lowerLimit = 0.0
       float physics:upperLimit = 0.7
       ...
   }

   def Joint "right_outer_knuckle_joint"
   {
       float physics:lowerLimit = -0.7
       float physics:upperLimit = 0.0
       ...
   }
   ```

4. **Use Joint Limits**
   - `joint_closed_positions` should be near the limit
   - For left finger: close to `upperLimit`
   - For right finger: close to `lowerLimit`
   - Leave small margin (e.g., 0.628 instead of 0.7)

---

## 2. Understanding `action_deltas`

### What It Is
The amount (in radians) the joint position changes per control action.

### Current Values
```python
action_deltas=np.array([-0.2, 0.2])
```
- Left finger: -0.2 rad per step (closing direction)
- Right finger: +0.2 rad per step (closing direction)

---

## How to Find `action_deltas`

### Concept

`action_deltas` controls **gripper speed**:
- **Larger value** → Faster gripper motion
- **Smaller value** → Slower, more controlled motion

It's the increment applied each time the gripper is commanded to close or open.

---

### Method 1: Empirical Testing (Recommended)

1. **Start with Default**
   ```python
   action_deltas=np.array([-0.2, 0.2])  # 0.2 rad ≈ 11.5 degrees per step
   ```

2. **Test Gripper Performance**
   Run your pick-place example and observe:
   - Does gripper close fast enough?
   - Does it overshoot (too fast)?
   - Does it grip reliably?

3. **Adjust Based on Observations**

   | Observation | Problem | Solution |
   |-------------|---------|----------|
   | Gripper closes too slowly | action_delta too small | Increase to 0.3 or 0.4 |
   | Gripper overshoots/bounces | action_delta too large | Decrease to 0.1 or 0.15 |
   | Gripper doesn't grip firmly | Closes too fast | Decrease action_delta |
   | Gripper damages object | Too much force | Decrease action_delta + increase wait time |

4. **Fine-Tune**
   ```python
   # Too slow
   action_deltas=np.array([-0.1, 0.1])  # 5.7° per step

   # Balanced (default)
   action_deltas=np.array([-0.2, 0.2])  # 11.5° per step

   # Fast
   action_deltas=np.array([-0.3, 0.3])  # 17.2° per step
   ```

---

### Method 2: Calculate from Desired Closing Time

1. **Determine Desired Closing Time**
   - Example: Want gripper to close in 0.5 seconds

2. **Know Your Control Frequency**
   - Isaac Sim default: 60 Hz (60 steps per second)
   - In 0.5 seconds: 30 control steps

3. **Calculate Required Delta**
   ```python
   # To close from 0 to 0.628 rad in 30 steps
   action_delta = 0.628 / 30 = 0.021 rad per step
   ```

4. **Account for Controller**
   - PickPlaceController doesn't apply delta every step
   - It uses `events_dt[3]` for close duration (default 0.05 seconds)
   - Adjust accordingly:

   ```python
   close_duration = 0.05  # seconds (from events_dt[3])
   control_freq = 60  # Hz
   steps_to_close = close_duration * control_freq = 3 steps

   action_delta = 0.628 / 3 = 0.21 ≈ 0.2 rad per step
   ```

---

### Method 3: Based on Gripper Specifications

OnRobot RG6 specifications:
- **Closing time**: 0.5 - 1.0 seconds (full stroke)
- **Control frequency**: 60 Hz
- **Full stroke**: 0.628 rad

```python
# For 0.5 second closing
steps = 0.5 * 60 = 30
action_delta = 0.628 / 30 = 0.021 rad

# For 1.0 second closing
steps = 1.0 * 60 = 60
action_delta = 0.628 / 60 = 0.010 rad

# Practical value (accounts for controller timing)
action_delta ≈ 0.2 rad
```

---

## Relationship Between Parameters

### Is `joint_closed_positions` Related to `cube_size`?

**No, they are independent.**

| Parameter | Purpose | Depends On |
|-----------|---------|------------|
| `joint_closed_positions` | Maximum closure angle | **Gripper geometry** (linkage mechanism, joint limits) |
| `cube_size` | Size of object to grasp | **Your task** (what you're picking up) |
| `action_deltas` | Gripper speed | **Desired closing time** and control frequency |

### Why They Seem Related

The gripper will **automatically stop** when it contacts the cube, regardless of `joint_closed_positions`:

```
cube_size = 0.0515 m (5.15 cm)

Gripper behavior:
1. Start closing from 0.0 rad
2. Fingers approach cube
3. Touch cube at ~0.3 rad (depends on cube position)
4. Contact detected → STOP closing
5. Even though joint_closed_positions = 0.628 rad
```

**Key Point**: `joint_closed_positions` is the **maximum** the gripper CAN close, not how much it WILL close for a specific object.

---

## Practical Guidelines

### For `joint_closed_positions`

1. **Start with gripper specs**
   - Check manufacturer documentation
   - Use joint limits from USD file
   - Typical parallel grippers: 0.5 - 0.7 rad (30-40°)

2. **Test in simulation**
   - Manually close gripper in Isaac Sim
   - Note when fingers are fully closed (touching)
   - Use that value

3. **Leave safety margin**
   - If joint limit is 0.7, use 0.628
   - Prevents hitting hard stops
   - Reduces wear on simulation joints

### For `action_deltas`

1. **Start with 0.2 rad** (good default)

2. **Tune based on performance**
   - Object slips → slower (0.1 - 0.15)
   - Too slow → faster (0.25 - 0.3)
   - Fragile objects → slower + increase wait time

3. **Consider cube size indirectly**
   - **Small objects** (1-3 cm): Slower deltas (0.1-0.15) for precision
   - **Medium objects** (5-10 cm): Default (0.2)
   - **Large objects** (>10 cm): Can use faster (0.25-0.3)

---

## Example Configurations

### Small Delicate Object (1cm cube)

```python
gripper = ParallelGripper(
    end_effector_prim_path="/World/cobotta/onrobot_rg6_base_link",
    joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],
    joint_opened_positions=np.array([0, 0]),
    joint_closed_positions=np.array([0.628, -0.628]),  # Same max closure
    action_deltas=np.array([-0.1, 0.1])  # Slower for precision
)
```

### Large Heavy Object (15cm cube)

```python
gripper = ParallelGripper(
    end_effector_prim_path="/World/cobotta/onrobot_rg6_base_link",
    joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],
    joint_opened_positions=np.array([0, 0]),
    joint_closed_positions=np.array([0.628, -0.628]),  # Same max closure
    action_deltas=np.array([-0.3, 0.3])  # Faster, firm grip
)
```

### Current Configuration (5.15cm cube)

```python
gripper = ParallelGripper(
    end_effector_prim_path="/World/cobotta/onrobot_rg6_base_link",
    joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],
    joint_opened_positions=np.array([0, 0]),
    joint_closed_positions=np.array([0.628, -0.628]),  # 36° max
    action_deltas=np.array([-0.2, 0.2])  # Balanced speed
)
```

---

## Verification Checklist

After setting these parameters, verify:

- [ ] Gripper opens fully (fingers don't collide)
- [ ] Gripper closes completely (can touch if no object)
- [ ] Gripper stops at object contact (doesn't crush it)
- [ ] Closing speed is appropriate (not too fast/slow)
- [ ] Gripper grips reliably (object doesn't slip)
- [ ] No joint limit warnings in console

---

## Quick Reference

### To Find `joint_closed_positions`:

1. Load robot in Isaac Sim
2. Manually close gripper joints
3. Note angles when fully closed
4. Use those values (with small margin)

**NOT related to cube_size** - this is maximum gripper closure.

### To Find `action_deltas`:

1. Start with 0.2 rad
2. Test pick-and-place
3. Adjust based on:
   - Too slow → increase
   - Too fast/overshoots → decrease
   - Object slips → decrease

**Indirectly related to cube_size** - smaller objects need slower, more precise motion.

---

## Advanced: Different Gripper Types

If using a different gripper (not RG6), the process is the same but values differ:

### Robotiq 2F-85 Example
```python
joint_closed_positions=np.array([0.8, -0.8])  # 46° for full 85mm stroke
action_deltas=np.array([-0.15, 0.15])  # Slower for precision
```

### Custom Gripper Example
```python
# Step 1: Find max closure in Isaac Sim → 0.5 rad
# Step 2: Test speeds → 0.25 rad works well

joint_closed_positions=np.array([0.5, -0.5])
action_deltas=np.array([-0.25, 0.25])
```

---

## Troubleshooting

### Gripper Won't Close Enough
- Increase `joint_closed_positions` (e.g., 0.628 → 0.7)
- Check if hitting joint limits
- Verify finger geometry isn't colliding

### Gripper Closes Too Much (Crushes Object)
- Add wait time in `events_dt[2]` (stabilization)
- Decrease `action_deltas` for gentler closing
- Check contact/collision detection settings

### Gripper Opens/Closes Too Fast
- Decrease `action_deltas`
- Increase `events_dt[3]` (close duration)
- Increase `events_dt[8]` (open duration)

### Object Slips During Transport
- Decrease `action_deltas` (slower, firmer grip)
- Increase `events_dt[4]` (wait after grasp)
- Check gripper friction properties

---

## Summary

1. **`joint_closed_positions`** (0.628, -0.628):
   - Find by manually closing gripper in Isaac Sim
   - Based on gripper geometry, NOT cube size
   - Represents maximum possible closure

2. **`action_deltas`** (0.2, -0.2):
   - Find by empirical testing
   - Start with 0.2, adjust for performance
   - Controls gripper speed
   - Indirectly affected by object size (small = slower)

3. **Cube size** (0.0515):
   - Independent parameter
   - Gripper automatically stops at contact
   - Doesn't determine joint positions

**Rule of Thumb**:
- Set `joint_closed_positions` once (based on gripper)
- Tune `action_deltas` per task (based on object)
- Change `cube_size` as needed (task definition)
