# Robot Pick and Place Control Scripts

This directory contains Python scripts to control two Kuka KR210 robot arms for automated pick and place operations in NVIDIA Isaac Sim.

## Overview

### Task Sequence
1. **Robot 2 (kr210_l150_02)**: Picks `geo_boxLid_01` and places it on `Danny` table
2. **Robot 1 (kr210_l150_01)**: Picks `geo_cube_01` and places it into `grp_boxBody_01`
3. **Both robots**: Return to original positions

## Files

### 1. `robot_pick_place.py`
Basic implementation with simple motion control.
- Good for understanding the basic flow
- Uses simple trajectory planning
- Suitable for initial testing

### 2. `robot_pick_place_advanced.py` (Recommended)
Advanced implementation with better motion planning.
- Smoother trajectories with interpolation
- Better structured code
- More detailed logging and status updates
- Recommended for actual use

## Prerequisites

### Software Requirements
- NVIDIA Isaac Sim 2023.1.0 or later
- Python 3.10+
- USD file: `po_wiwynn_test_v0002.usda`

### Required Isaac Sim Extensions
```python
omni.isaac.core
omni.isaac.manipulators
omni.isaac.motion_generation (for advanced IK)
```

## Installation

1. **Install NVIDIA Isaac Sim**
   - Download from: https://developer.nvidia.com/isaac-sim
   - Follow installation instructions

2. **Setup Python Environment**
   ```bash
   # Use Isaac Sim's Python
   # Typically located at: ~/.local/share/ov/pkg/isaac_sim-*/python.sh
   ```

3. **Verify USD File Path**
   - Ensure `po_wiwynn_test_v0002.usda` is in the correct location
   - Update path in scripts if needed

## Usage

### Running the Basic Script
```bash
# Navigate to Isaac Sim directory
cd /path/to/isaac_sim

# Run with Isaac Sim's Python
./python.sh /path/to/po_wiwynn_test/robot_pick_place.py
```

### Running the Advanced Script (Recommended)
```bash
# Navigate to Isaac Sim directory
cd /path/to/isaac_sim

# Run with Isaac Sim's Python
./python.sh /path/to/po_wiwynn_test/robot_pick_place_advanced.py
```

### Windows Example
```cmd
cd C:\Users\YourName\AppData\Local\ov\pkg\isaac_sim-2023.1.0
python.bat D:\poc\po_wiwynn_test\robot_pick_place_advanced.py
```

### Linux Example
```bash
cd ~/.local/share/ov/pkg/isaac_sim-2023.1.0
./python.sh ~/poc/po_wiwynn_test/robot_pick_place_advanced.py
```

## Configuration

### Adjusting Positions

Edit these parameters in the script to adjust pick/place locations:

```python
# In execute_full_sequence() method

# Danny table placement height
danny_target[2] += 0.4  # Adjust this value

# Box body placement height
box_target[2] += 0.2  # Adjust this value

# Approach height for pick/place
approach_height = 0.3  # Adjust in method calls
```

### Adjusting Motion Speed

```python
# In move_to_joint_positions() method
num_steps = 100  # Increase for slower, smoother motion
                 # Decrease for faster motion
```

## Implementation Notes

### Current Limitations

1. **Inverse Kinematics**: The current implementation uses placeholder IK.
   - For production, integrate with Isaac Sim's Lula IK solver
   - Or use RMPflow for motion planning

2. **Gripper Control**: Gripper open/close are placeholders.
   - Implement actual gripper joint control based on your gripper type
   - Consider force control for delicate objects

3. **Collision Avoidance**: Not implemented.
   - Add collision detection for safe operation
   - Use Isaac Sim's collision APIs

### Recommended Improvements

#### 1. Add Lula IK Solver
```python
from omni.isaac.motion_generation import LulaKinematicsSolver

# Initialize IK solver
kinematics_solver = LulaKinematicsSolver(
    robot_description_path="robot_description.yaml",
    urdf_path="robot.urdf"
)

# Compute IK
joint_positions = kinematics_solver.compute_inverse_kinematics(
    target_position=target_pos,
    target_orientation=target_rot
)
```

#### 2. Add RMPflow Motion Planning
```python
from omni.isaac.motion_generation import ArticulationMotionPolicy

# Create motion policy
motion_policy = ArticulationMotionPolicy(
    robot_articulation=robot,
    end_effector_name="tool0"
)

# Get collision-free trajectory
trajectory = motion_policy.get_next_articulation_action(
    target_position=target_pos
)
```

#### 3. Implement Physics-Based Grasping
```python
from omni.isaac.core.utils.physics import attach_rigid_prim

# Attach object to end effector during pick
attach_rigid_prim(
    object_prim_path="/World/geo_boxLid_01",
    parent_prim_path="/World/kr210_l150_02/tool0"
)

# Detach during place
detach_rigid_prim(object_prim_path="/World/geo_boxLid_01")
```

## Troubleshooting

### Issue: USD File Not Found
**Solution**: Update the path in the script
```python
usd_path = "D:/poc/po_wiwynn_test/po_wiwynn_test_v0002.usda"
# Change to your actual path
```

### Issue: Robots Not Moving
**Possible causes**:
1. Physics not enabled - check PhysicsScene in USD
2. Joint limits - verify robot joint ranges
3. Simulation not stepping - ensure `world.step()` is called

### Issue: Objects Falling Through
**Solution**: Verify collision properties in USD file
- Check `PhysicsCollisionAPI` is applied
- Verify collision shapes are correct
- Ensure ground plane has collision

### Issue: Gripper Not Grasping
**Solution**: Implement actual gripper control
- Identify gripper joints in robot USD
- Apply joint forces/positions for open/close
- Consider using surface gripper or constraint-based grasping

## Scene Information

### Robot Positions
- **kr210_l150_01**: `(1.867, -0.719, 0)`
- **kr210_l150_02**: `(-2.291, 0, 0)`

### Object Positions
- **geo_boxLid_01**: `(0, 0, 1.435)` - Box lid to be placed on Danny
- **geo_cube_01**: `(0, -0.828, 1.115)` - Cube to be placed in box

### Target Positions
- **Danny table**: `(0, -0.184, 0)` - Table surface
- **grp_boxBody_01**: `(0, 0, 1.025)` - Box container

## Performance Optimization

### For Faster Execution
```python
# Reduce simulation substeps
self.world.reset(soft=True)

# Reduce render updates
if i % 5 == 0:  # Update every 5 steps
    simulation_app.update()

# Use headless mode for batch processing
config = {"headless": True}
```

### For Better Visualization
```python
# Increase window resolution
config = {
    "headless": False,
    "width": 2560,
    "height": 1440,
}

# Add camera tracking
from omni.isaac.core.utils.viewports import set_camera_view
set_camera_view(
    eye=[3, 3, 2],
    target=[0, 0, 1],
    camera_prim_path="/OmniverseKit_Persp"
)
```

## Additional Resources

- [Isaac Sim Documentation](https://docs.omniverse.nvidia.com/isaacsim/latest/index.html)
- [Isaac Core API](https://docs.omniverse.nvidia.com/isaacsim/latest/core_api_tutorials/index.html)
- [Motion Generation Tutorial](https://docs.omniverse.nvidia.com/isaacsim/latest/advanced_tutorials/tutorial_motion_generation.html)
- [Manipulation Tutorial](https://docs.omniverse.nvidia.com/isaacsim/latest/manipulation_tutorial.html)

## License

This code is provided as-is for the Wiwynn POC project.

## Support

For issues or questions:
1. Check Isaac Sim forums: https://forums.developer.nvidia.com/c/omniverse/simulation/69
2. Review Isaac Sim documentation
3. Contact the development team

---

**Last Updated**: 2025-11-24
**Isaac Sim Version**: 2023.1.0+
**Status**: Development/POC
