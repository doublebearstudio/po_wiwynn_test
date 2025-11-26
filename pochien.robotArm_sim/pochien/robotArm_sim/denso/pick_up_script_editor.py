"""
Pick and Place - Script Editor Version (Simplified)

This script uses the simple PickPlaceSceneSetup class instead of BaseTask.
No World.add_task(), no complex initialization - just straightforward setup.

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
initial_position = np.array([-0.5, 0.4, 0.125])
target_position = np.array([-0.6, -0.5, 0.135])
custom_usd_path = "D:/poc/po_wiwynn_test/tst_cylinder01.usda"

# ============================================================================
# 2. Setup Scene
# ============================================================================
print("\n→ Setting up scene...")

# Create setup helper
setup = PickPlaceSceneSetup(
    custom_usd_path=custom_usd_path,
    object_initial_position=initial_position,
    target_position=target_position
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
was_playing = False

def simulation_step(step_size):
    """Called every physics step"""
    global initialized, was_playing

    # Detect timeline stop/start cycle
    is_playing = timeline.is_playing()

    if not is_playing:
        # Timeline stopped - reset initialization flag
        if was_playing:
            initialized = False
            was_playing = False
        return

    # Timeline is playing
    was_playing = True

    # Initialize/re-initialize when timeline starts
    if not initialized:
        try:
            robot.initialize()

            # Wait for physics view to be created
            joint_positions = robot.get_joint_positions()
            if joint_positions is None:
                # Physics view not ready yet, try again next frame
                return

            # Physics view is ready, now initialize controller
            controller.reset()
            initialized = True
            print("\n✓ System initialized")
            print("="*70)
            print("RUNNING PICK AND PLACE")
            print("="*70)
        except Exception as e:
            # Wait for physics context
            print(f"Waiting for initialization: {e}")
            return

    # Main control loop
    try:
        # Get joint positions
        joint_positions = robot.get_joint_positions()

        # Safety check (shouldn't happen after initialization, but just in case)
        if joint_positions is None:
            print("Warning: Physics view lost, re-initializing...")
            initialized = False
            return

        # Get observations
        observations = setup.get_observations(robot)

        # Compute actions
        actions = controller.forward(
            picking_position=observations["pickup_object"]["position"],
            placing_position=observations["pickup_object"]["target_position"],
            current_joint_positions=joint_positions,
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
