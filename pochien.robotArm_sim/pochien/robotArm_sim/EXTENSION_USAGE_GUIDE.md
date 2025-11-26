# Robot Pick-and-Place Extension - Usage Guide

## Overview

The extension has been successfully integrated with full robot pick-and-place functionality for the Denso Cobotta robot in your existing USD scene.

## What Was Implemented

### File Updated
- **`extension.py`** - Complete robot pickup integration (518 lines)

### Key Features

1. **Scene Loading**
   - Automatically loads: `D:\poc\po_wiwynn_test\po_wiwynn_test_v0003.usda`
   - Uses existing robot prim: `/World/cobotta_pro_900`
   - Uses existing cube prim: `/World/geo_cube_01`
   - Preserves existing PhysicsScene and physics properties

2. **Pick-and-Place Operation**
   - **Pickup Position**: (1.0, 0.0, 0.025) - Current cube location
   - **Place Position**: (0.0, 1.0, 0.025) - Target drop location
   - **Controller**: PickPlaceController with RMPflow motion planning
   - **Auto Home Return**: Robot automatically returns to initial position after task completion

3. **UI Controls**
   - **Start Button**: Loads scene (first time) and begins pick-and-place
   - **Stop Button**: Immediately halts simulation and pauses physics
   - **Reset Scene**: Reloads USD file and reinitializes everything
   - **Return to Home**: Manually moves robot back to home position

4. **Status Display**
   - Real-time status updates with color coding:
     - Green: Normal operation
     - Red: Errors
   - Collapsible sections for scene configuration and task parameters

## How to Use

### Step 1: Enable the Extension in Isaac Sim

1. Open Isaac Sim
2. Go to **Window → Extensions**
3. Search for your extension name
4. Enable it (toggle switch)
5. The "Robot Pick & Place Control" window should appear

### Step 2: Run Pick-and-Place

1. Click **"Start Pick & Place"** button
2. Watch the status updates:
   - "Loading scene file..."
   - "Creating simulation world..."
   - "Configuring robot..."
   - "Initializing physics..."
   - "Ready - Click Start to begin"
   - (Auto-starts after 1 second)
   - "Running: Pick and place in progress..."

3. The robot will:
   - Move to pickup position (1, 0, 0.025)
   - Grasp the cube
   - Transport to place position (0, 1, 0.025)
   - Release the cube
   - Return to home position

4. After completion:
   - Status: "Complete! Robot returned to home. Click Start to repeat."
   - Click "Start" again to repeat the task

### Step 3: Control Operations

**Stop Button**
- Halts simulation immediately
- Pauses physics
- Keeps robot in current position
- Re-enables Start button

**Reset Scene**
- Reloads the USD file from disk
- Clears and recreates World
- Reinitializes robot and controller
- Use if scene becomes corrupted or you want fresh start

**Return to Home**
- Manually moves robot to home position
- Only works when robot is initialized
- Use if robot gets stuck or you want to reset position

## Architecture

### Flow Diagram

```
Extension Startup
    ↓
Build UI → Initialize State Variables
    ↓
User Clicks "Start"
    ↓
_initialize_scene_async()
    ├─ Load USD scene
    ├─ Create World (no ground plane)
    ├─ Setup robot + gripper
    ├─ Reset world physics
    └─ Store home position
    ↓
_start_simulation()
    ├─ Reset PickPlaceController
    ├─ Subscribe to physics steps
    └─ Start physics (World.play())
    ↓
_on_physics_step() [Every frame]
    ├─ Get current joint positions
    ├─ Compute actions via controller.forward()
    ├─ Apply actions via articulation_controller
    └─ Check if done
    ↓
Task Complete
    ├─ Wait 2 seconds
    ├─ Return to home position
    └─ Stop simulation
```

### Key Components

1. **World** (`omni.isaac.core.World`)
   - Manages simulation context
   - Created without ground plane (already in USD)
   - Controls physics play/pause

2. **SingleManipulator** (`isaacsim.robot.manipulators.manipulators`)
   - Wraps the robot articulation
   - Provides high-level interface
   - Manages end-effector

3. **ParallelGripper** (`isaacsim.robot.manipulators.grippers`)
   - OnRobot RG6 gripper configuration
   - Open position: [0, 0]
   - Closed position: [0.628, -0.628]

4. **PickPlaceController** (`denso/pick_place_controller.py`)
   - 10-state state machine
   - RMPflow motion planning
   - Automatic gripper coordination

5. **ArticulationController**
   - Low-level joint control
   - Applies actions computed by PickPlaceController

## Configuration

### Modifying Positions

Edit these lines in `extension.py`:

```python
# Task configuration
self._pickup_position = np.array([1.0, 0.0, 0.025])  # Change pickup
self._place_position = np.array([0.0, 1.0, 0.025])   # Change placement
self._end_effector_offset = np.array([0, 0, 0.25])   # Gripper offset
```

### Modifying Scene Paths

```python
# Scene paths
self._scene_path = r"D:\poc\po_wiwynn_test\po_wiwynn_test_v0003.usda"
self._robot_prim_path = "/World/cobotta_pro_900"
self._cube_prim_path = "/World/geo_cube_01"
```

### Adjusting Motion Speed

The PickPlaceController timing is configured in `denso/pick_place_controller.py` with the `events_dt` parameter. See `PICK_PLACE_CONTROLLER_BREAKDOWN.md` for details.

## Troubleshooting

### Extension Doesn't Appear
- Check that extension.toml has correct python.module path
- Verify extension is enabled in Extensions window
- Check console for import errors

### "Scene file not found" Error
- Verify path: `D:\poc\po_wiwynn_test\po_wiwynn_test_v0003.usda`
- Check that file exists at that location
- Update `self._scene_path` if file moved

### "Robot not initialized" Error
- Click "Reset Scene" to reinitialize
- Check that robot prim `/World/cobotta_pro_900` exists in USD file
- Verify robot has correct structure (joints, end-effector)

### "PickPlaceController not available" Warning
- Verify `denso/pick_place_controller.py` exists
- Check that denso directory is in correct location
- Look for import errors in console

### Robot Doesn't Move
- Check that physics is playing (status should show "Running")
- Verify gripper is initialized (check console logs)
- Try clicking "Stop" then "Start" again

### Gripper Doesn't Grasp
- Verify gripper joint names match USD file
- Check gripper positions (opened/closed)
- Adjust `events_dt[2]` (wait before grasp) in pick_place_controller.py
- See `PICK_PLACE_CONTROLLER_BREAKDOWN.md` for tuning guide

### Robot Moves Too Fast/Slow
- Adjust PickPlaceController timing in `denso/pick_place_controller.py`
- See Event Timing section in `PICK_PLACE_CONTROLLER_BREAKDOWN.md`
- Default timings are conservative for stability

### Scene Corruption
- Click "Reset Scene" to reload from USD file
- If problem persists, restart Isaac Sim
- Check USD file integrity

## Implementation Details

### Physics Step Callback

The core control loop runs every physics step:

```python
def _on_physics_step(self, dt):
    # Get current state
    current_joint_positions = self._robot.get_joint_positions()

    # Compute next action
    actions = self._controller.forward(
        picking_position=self._pickup_position,
        placing_position=self._place_position,
        current_joint_positions=current_joint_positions,
        end_effector_offset=self._end_effector_offset
    )

    # Apply action
    self._articulation_controller.apply_action(actions)

    # Check completion
    if self._controller.is_done():
        # Return to home
```

### Async Scene Initialization

Scene loading is asynchronous to prevent UI blocking:

```python
async def _initialize_scene_async(self):
    open_stage(self._scene_path)
    await asyncio.sleep(0.5)  # Let stage load

    self._world = World(...)
    await self._setup_robot_async()
    await self._world.reset_async()

    # Auto-start after initialization
    await asyncio.sleep(1.0)
    self._start_simulation()
```

### State Management

- `_is_running`: True when simulation is active
- `_task_complete`: Prevents multiple home returns
- `_subscription`: Physics step callback handle
- `_home_joint_positions`: Stored after world reset

## Related Documentation

- **`PICK_UP_EXAMPLE_BREAKDOWN.md`** - Original standalone example
- **`PICK_PLACE_BREAKDOWN.md`** - Robot configuration details
- **`PICK_PLACE_CONTROLLER_BREAKDOWN.md`** - Controller state machine and timing
- **`PHYSICS_INITIALIZATION_GUIDE.md`** - Physics system setup

## Next Steps

1. **Test the Extension**
   - Enable in Isaac Sim
   - Click Start and verify operation
   - Check console for any errors

2. **Tune Performance**
   - Adjust pickup/place positions if needed
   - Modify controller timing for your application
   - Optimize gripper grasp parameters

3. **Extend Functionality**
   - Add multiple pickup/place positions
   - Implement object detection
   - Add error recovery logic
   - Create custom state sequences

## Support

If you encounter issues:
1. Check console output for detailed error messages
2. Review status display in extension window
3. Consult the breakdown documentation files
4. Verify USD scene structure matches expected prims

---

**Integration Complete**: Your extension is now ready to control robot pick-and-place operations in the existing USD scene!
