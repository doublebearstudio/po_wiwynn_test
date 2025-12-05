"""
Pick and Place - Standalone Script Editor Version (Simplified)

This is a test file only used for setting up a single pick and place event directly from Script Editor.
Not a module and nor should be used in any official scenario.

Usage:
1. Open Isaac Sim
2. Open Script Editor (Window > Script Editor)
3. Paste this entire script
4. Run it
5. Press PLAY in timeline
"""

import sys
import numpy as np

# Add module path (adjust if needed)
module_path = r"D:/poc/po_wiwynn_test/pochien.robotArm_sim/pochien/robotArm_sim/denso"
if module_path not in sys.path:
    sys.path.append(module_path)

# Import simple setup class
from simple_pick_place_setup import PickPlaceSceneSetup
from pick_place_controller import PickPlaceController

# Import Isaac Sim modules
from isaacsim.core.utils.stage import add_reference_to_stage
from omni.timeline import get_timeline_interface
import omni.physx as _physx

print("="*70)
print("PICK AND PLACE SETUP - SIMPLIFIED")
print("="*70)

# ============================================================================
# 1. Define Positions
# ============================================================================
robot_position = np.array([0.0, 0.0, 0.0])
initial_position = np.array([-0.5, 0.4, 0.04])
target_position = np.array([-0.6, -0.5, 0.04])
custom_usd_path = "D:/poc/po_wiwynn_test/tst_cylinder01.usda"

# ============================================================================
# 2. Setup Scene
# ============================================================================
print("\n→ Setting up scene...")

# Create setup helper
setup = PickPlaceSceneSetup(
    custom_usd_path=custom_usd_path,
    object_initial_position=initial_position,
    target_position=target_position,
    robot_position=robot_position
)

# Add table (optional)
table_path = "D:/poc/po_wiwynn_test/prp_table01.usda"
add_reference_to_stage(usd_path=table_path, prim_path="/World/Table")

# Create robot
print("  → Creating robot...")
robot = setup.create_robot()

# Create pickup object
print("  → Creating pickup object...")
setup.create_object(mass=0.01)  # 10 grams

# Create target marker
print("  → Creating target marker...")
setup.create_target_marker()

print("✓ Scene setup complete")

# ============================================================================
# 3. Initialize Controller
# ============================================================================
print("\n→ Initializing controller...")

controller = PickPlaceController(
    name="pick_place_controller",
    robot_articulation=robot,
    gripper=robot.gripper
)

articulation_controller = robot.get_articulation_controller()

print("✓ Controller ready")

# ============================================================================
# 4. Control Loop Setup
# ============================================================================
timeline = get_timeline_interface()
initialized = False

def simulation_step(step_size):
    """Called every physics step"""
    global initialized

    # One-time initialization when timeline starts
    if not initialized:
        try:
            robot.initialize()
            controller.reset()
            initialized = True
            print("\n✓ System initialized")
            print("="*70)
            print("RUNNING PICK AND PLACE")
            print("="*70)
        except Exception as e:
            # Wait for physics context
            return

    # Main control loop
    try:
        # Get observations
        observations = setup.get_observations(robot)

        # Transform world coordinates to robot-local coordinates
        # The controller expects positions relative to the robot's base frame
        object_world_pos = observations["pickup_object"]["position"]
        target_world_pos = observations["pickup_object"]["target_position"]

        # Subtract robot base position to get robot-local coordinates
        object_local_pos = object_world_pos - robot_position
        target_local_pos = target_world_pos - robot_position

        # Compute actions using robot-local coordinates
        actions = controller.forward(
            picking_position=object_local_pos,
            placing_position=target_local_pos,
            current_joint_positions=observations["cobotta_robot"]["joint_positions"],
            end_effector_offset=np.array([0, 0, 0.25]),
        )

        # Check completion
        if controller.is_done():
            print("✓ Pick and place complete!")

        # Apply actions
        articulation_controller.apply_action(actions)

    except Exception as e:
        print(f"Error in control loop: {e}")

# Subscribe to physics updates
physics_subscription = _physx.get_physx_interface().subscribe_physics_step_events(simulation_step)

print("\n" + "="*70)
print("READY")
print("="*70)
print("→ Press ▶ PLAY button to start")
print("→ To stop: physics_subscription.unsubscribe()")
print("="*70)
