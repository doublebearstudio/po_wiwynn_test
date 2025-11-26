# Robot Arm Configuration Comparison

## Complete comparison between working example and extension

This document compares **pick_up_example.py** (working) vs **extension.py** (current) to identify differences.

---

## 1. World Initialization

| Aspect | Working Example | Extension | Match |
|--------|----------------|-----------|-------|
| **Creation** | `World(stage_units_in_meters=1.0)` | `World(stage_units_in_meters=1.0)` | ✓ |
| **Ground Plane** | Auto-added (default=True) | `add_ground_plane=False` (line 318) | ✗ |
| **Physics Scene** | Auto-added by World.reset() | Uses existing from USD | Different |
| **Reset Method** | `my_world.reset()` (sync) | `await self._world.reset_async()` (async) | ✓ |

**Issue Found**: Extension disables ground plane but working example uses it.

---

## 2. Robot Loading

| Aspect | Working Example | Extension | Match |
|--------|----------------|-----------|-------|
| **Loading Method** | Via PickPlace task | Direct USD load via `open_stage()` | Different |
| **Robot Path** | `/World/cobotta` | `/World/cobotta_pro_900` | Different |
| **USD Source** | PickPlace.set_robot() adds reference | Pre-existing in scene file | Different |

**Key Difference**: Working example creates a fresh robot via task, extension uses existing robot from USD file.

---

## 3. Gripper Configuration

| Parameter | Working Example | Extension | Match |
|-----------|----------------|-----------|-------|
| **end_effector_prim_path** | `/World/cobotta/onrobot_rg6_base_link` | `{robot_prim}/onrobot_rg6_base_link` | ✓ |
| **joint_prim_names** | `["finger_joint", "right_outer_knuckle_joint"]` | `["finger_joint", "right_outer_knuckle_joint"]` | ✓ |
| **joint_opened_positions** | `[0, 0]` | `[0, 0]` | ✓ |
| **joint_closed_positions** | `[0.628, -0.628]` | `[0.628, -0.628]` | ✓ |
| **action_deltas** | `[-0.2, 0.2]` | `[-0.2, 0.2]` | ✓ |

**Status**: Gripper configuration is identical ✓

---

## 4. Robot Manipulator Configuration

| Parameter | Working Example | Extension | Match |
|-----------|----------------|-----------|-------|
| **prim_path** | `/World/cobotta` | `/World/cobotta_pro_900` | Different paths |
| **name** | `"cobotta_robot"` | `"cobotta_robot"` | ✓ |
| **end_effector_prim_name** | `"onrobot_rg6_base_link"` | `"onrobot_rg6_base_link"` | ✓ |
| **gripper** | Passed via parameter | Passed via parameter | ✓ |

**Issue**: Different prim paths, but this should be OK if both robots have same structure.

---

## 5. Default Joint Positions

| Joint Indices | Working Example | Extension | Match |
|---------------|----------------|-----------|-------|
| **0-5 (arm)** | 0.0 | 0.0 | ✓ |
| **6 (finger_joint)** | 0.0 | 0.0 | ✓ |
| **7 (left_outer_knuckle)** | **0.628** | **0.628** | ✓ |
| **8 (left_inner_knuckle)** | **0.628** | **0.628** | ✓ |
| **9-11 (other gripper)** | 0.0 | 0.0 | ✓ |
| **Set Method** | `set_joints_default_state(positions=...)` | `set_joints_default_state(positions=...)` | ✓ |

**Status**: Default joint positions match ✓ (after recent fix)

---

## 6. PickPlaceController Initialization

| Parameter | Working Example | Extension | Match |
|-----------|----------------|-----------|-------|
| **name** | `"controller"` | `"controller"` | ✓ |
| **robot_articulation** | `my_denso` (from scene) | `self._robot` (created) | ✓ |
| **gripper** | `my_denso.gripper` | `self._gripper` | Different |
| **events_dt** | `None` (uses defaults) | `None` (uses defaults) | ✓ |

**Critical Difference**:
- Working: `gripper=my_denso.gripper` (gets gripper from robot object)
- Extension: `gripper=self._gripper` (separate gripper object)

---

## 7. Gripper Initialization

| Aspect | Working Example | Extension | Match |
|--------|----------------|-----------|-------|
| **When** | Before controller creation | After world reset | Different |
| **Method** | Implicit (via task) | Explicit with callbacks | Different |
| **Callbacks** | Not shown (handled by task) | Manually provided | Different |

**Working Example Flow**:
```python
my_task = PickPlace(...)  # Creates robot with gripper
my_world.add_task(my_task)
my_world.reset()
my_denso = my_world.scene.get_object("cobotta_robot")
# Gripper is already initialized as my_denso.gripper
```

**Extension Flow**:
```python
self._gripper = ParallelGripper(...)
self._robot = SingleManipulator(..., gripper=self._gripper)
self._world.scene.add(self._robot)
await self._world.reset_async()
# NOW initialize gripper with callbacks
self._gripper.initialize(
    articulation_apply_action_func=self._robot.apply_action,
    get_joint_positions_func=self._robot.get_joint_positions,
    set_joint_positions_func=self._robot.set_joint_positions,
    dof_names=self._robot.dof_names
)
```

**Issue**: Extension initializes gripper AFTER world reset, working example has it initialized through task.

---

## 8. Control Loop

| Aspect | Working Example | Extension | Match |
|--------|----------------|-----------|-------|
| **Loop Type** | `while simulation_app.is_running():` | Physics step callback | Different |
| **Step Method** | `my_world.step(render=True)` | Automatic (physics subscription) | Different |
| **Reset Check** | `if my_world.current_time_step_index == 0:` | Not checked | Different |
| **Controller Reset** | `my_controller.reset()` on first step | `my_controller.reset()` once at start | Different |

**Working Example**:
```python
while simulation_app.is_running():
    my_world.step(render=True)
    if my_world.is_playing():
        if my_world.current_time_step_index == 0:
            my_world.reset()
            my_controller.reset()

        observations = my_world.get_observations()
        actions = my_controller.forward(...)
        articulation_controller.apply_action(actions)
```

**Extension**:
```python
def _on_physics_step(self, dt):
    current_joint_positions = self._robot.get_joint_positions()
    actions = self._controller.forward(
        picking_position=self._pickup_position,
        placing_position=self._place_position,
        current_joint_positions=current_joint_positions,
        end_effector_offset=self._end_effector_offset
    )
    self._articulation_controller.apply_action(actions)
```

**Critical Difference**:
- Working example uses `my_world.get_observations()` to get positions dynamically
- Extension uses hardcoded `self._pickup_position` and `self._place_position`

---

## 9. Observations vs Direct Positions

| Aspect | Working Example | Extension | Match |
|--------|----------------|-----------|-------|
| **Picking Position** | From observations (dynamic) | Hardcoded array | ✗ |
| **Placing Position** | From observations (dynamic) | Hardcoded array | ✗ |
| **Current Joint Positions** | From observations | Direct call | ✓ |

**Working Example**:
```python
observations = my_world.get_observations()
picking_position = observations["cobotta_cube"]["position"]
placing_position = observations["cobotta_cube"]["target_position"]
current_joint_positions = observations["cobotta_robot"]["joint_positions"]
```

**Extension**:
```python
picking_position = self._pickup_position  # np.array([0.4, 0.3, 0.025])
placing_position = self._place_position   # np.array([-0.5, 0.3, 0.025])
current_joint_positions = self._robot.get_joint_positions()
```

**Issue**: Working example reads cube's actual position from scene, extension assumes cube is at hardcoded position.

---

## 10. End-Effector Offset

| Aspect | Working Example | Extension | Match |
|--------|----------------|-----------|-------|
| **Value** | `[0, 0, 0.25]` | `[0, 0, 0.25]` | ✓ |
| **Usage** | Passed to controller.forward() | Passed to controller.forward() | ✓ |

**Status**: End-effector offset matches ✓

---

## 11. Action Application

| Aspect | Working Example | Extension | Match |
|--------|----------------|-----------|-------|
| **Controller** | `articulation_controller` | `self._articulation_controller` | ✓ |
| **Method** | `apply_action(actions)` | `apply_action(actions)` | ✓ |
| **When** | Every frame in main loop | Every physics step | Equivalent |

**Status**: Action application is equivalent ✓

---

## Summary of Critical Differences

### 🔴 **Critical Issues**

1. **Controller Gripper Reference** (Line 378-382 in extension)
   - Working: `gripper=my_denso.gripper`
   - Extension: `gripper=self._gripper`
   - **Impact**: Controller might not have correct gripper reference

2. **Hardcoded Positions** (Line 495-499 in extension)
   - Working: Reads actual cube position from observations
   - Extension: Uses hardcoded pickup/place positions
   - **Impact**: Won't adapt if cube moves

3. **No Reset on Restart** (Missing in extension)
   - Working: Resets controller when `current_time_step_index == 0`
   - Extension: Only resets once at start
   - **Impact**: Can't repeat task without full reset

### 🟡 **Minor Differences**

4. **Ground Plane** (Line 318 in extension)
   - Working: Has ground plane
   - Extension: `add_ground_plane=False`
   - **Impact**: Might affect physics behavior

5. **Different Robot Paths**
   - Working: `/World/cobotta`
   - Extension: `/World/cobotta_pro_900`
   - **Impact**: Should be OK if structure is identical

---

## Recommended Fixes

### Fix 1: Use Robot's Gripper Reference

**Current (extension.py:378-382)**:
```python
if PickPlaceController:
    self._controller = PickPlaceController(
        name="controller",
        robot_articulation=self._robot,
        gripper=self._gripper  # ← Using separate gripper object
    )
```

**Should be**:
```python
if PickPlaceController:
    self._controller = PickPlaceController(
        name="controller",
        robot_articulation=self._robot,
        gripper=self._robot.gripper  # ← Use gripper from robot
    )
```

---

### Fix 2: Read Cube Position Dynamically

**Current (extension.py:495-499)**:
```python
actions = self._controller.forward(
    picking_position=self._pickup_position,  # Hardcoded
    placing_position=self._place_position,   # Hardcoded
    current_joint_positions=current_joint_positions,
    end_effector_offset=self._end_effector_offset
)
```

**Should be** (if cube is in scene):
```python
# Get cube's actual position from scene
cube_prim = self._world.stage.GetPrimAtPath(self._cube_prim_path)
from pxr import UsdGeom
xformable = UsdGeom.Xformable(cube_prim)
from pxr import Usd
time_code = Usd.TimeCode.Default()
world_transform = xformable.ComputeLocalToWorldTransform(time_code)
cube_position = np.array(world_transform.ExtractTranslation())

actions = self._controller.forward(
    picking_position=cube_position,  # Dynamic
    placing_position=self._place_position,
    current_joint_positions=current_joint_positions,
    end_effector_offset=self._end_effector_offset
)
```

**OR** (simpler - keep hardcoded but ensure cube is there):
```python
# Just make sure cube in USD file is at self._pickup_position
# This is acceptable for fixed demonstration
```

---

### Fix 3: Add Controller Reset on Restart

Add to extension.py in `_on_physics_step()`:

```python
def _on_physics_step(self, dt):
    if not self._is_running or not self._controller:
        return

    try:
        # Reset on simulation restart
        if self._world.current_time_step_index == 0:
            self._controller.reset()

        # ... rest of physics step code
```

---

### Fix 4: Enable Ground Plane (Optional)

**Current (extension.py:318)**:
```python
self._world = World(stage_units_in_meters=1.0)  # Has ground plane by default
```

This is actually fine now since you removed `add_ground_plane=False`.

---

## Configuration Checklist

Use this checklist to verify extension matches working example:

- [✓] Gripper configuration parameters match
- [✓] Default joint positions set (joints 7,8 = 0.628)
- [✓] End-effector offset = [0, 0, 0.25]
- [✗] Controller uses `robot.gripper` reference **← FIX THIS**
- [✓] Gripper initialized with callbacks after world reset
- [✗] Controller resets on time_step_index == 0 **← OPTIONAL FIX**
- [✓] Actions applied via articulation_controller
- [✓] Ground plane enabled (default behavior)

---

## Testing Recommendations

1. **First**: Apply Fix 1 (use robot.gripper)
2. **Test**: See if pick-and-place works
3. **If fails**: Check console for controller state messages
4. **If cube not grasped**: Verify cube is exactly at pickup position
5. **For repeatable tasks**: Apply Fix 3 (reset on restart)

---

## Key Insight

The most likely reason the extension doesn't work is:

**The controller is using `self._gripper` instead of `self._robot.gripper`**

In the working example, the gripper is accessed as `my_denso.gripper` which is the gripper attached to the SingleManipulator object. The extension creates a separate gripper and passes it to both the robot AND the controller, which might cause synchronization issues.

**Solution**: Change line 381 from `gripper=self._gripper` to `gripper=self._robot.gripper`
