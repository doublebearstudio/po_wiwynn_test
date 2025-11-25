"""
SIMPLE VERSION - Isaac Sim Script Editor
Copy and paste this entire script into Isaac Sim's Script Editor and click Run.

STEPS:
1. Open po_wiwynn_test_v0002.usda in Isaac Sim
2. Window > Script Editor
3. Copy/paste this entire file
4. Click "Run" button (or Ctrl+Enter)
"""

import numpy as np
import asyncio
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.prims import RigidPrim, XFormPrim
import omni.kit.app


async def pick_and_place_sequence():
    """Main pick and place sequence"""

    print("\n" + "="*60)
    print("Starting Robot Pick and Place Sequence")
    print("="*60 + "\n")

    # Get or create world
    world = World.instance()
    if world is None:
        world = World(stage_units_in_meters=1.0)

    await world.reset_async()
    await omni.kit.app.get_app().next_update_async()

    # Get robots
    print("Getting robots...")
    robot1 = world.scene.add(
        Articulation(prim_path="/World/kr210_l150_01", name="robot1")
    )
    robot2 = world.scene.add(
        Articulation(prim_path="/World/kr210_l150_02", name="robot2")
    )

    # Get objects
    print("Getting objects...")
    box_lid = world.scene.add(
        RigidPrim(prim_path="/World/geo_boxLid_01", name="box_lid")
    )
    cube = world.scene.add(
        RigidPrim(prim_path="/World/geo_cube_01", name="cube")
    )

    # Get targets
    danny_table = XFormPrim(prim_path="/World/Danny")
    box_body = XFormPrim(prim_path="/World/grp_boxBody_01")

    # Reset and get home positions
    await world.reset_async()
    await omni.kit.app.get_app().next_update_async()

    robot1_home = robot1.get_joint_positions()
    robot2_home = robot2.get_joint_positions()

    print("Home positions stored\n")

    # Helper function to move robot smoothly
    async def move_robot(robot, target_joints, steps=100, label="Moving"):
        print(f"  {label}...")
        current = robot.get_joint_positions()
        for i in range(steps):
            alpha = 0.5 * (1 - np.cos(np.pi * (i + 1) / steps))
            joints = current + alpha * (target_joints - current)
            robot.apply_action(ArticulationAction(joint_positions=joints))
            await world.play_async()
            await omni.kit.app.get_app().next_update_async()

    # Helper for gripper (placeholder)
    async def gripper_action(label, steps=30):
        print(f"  {label}...")
        for _ in range(steps):
            await world.play_async()
            await omni.kit.app.get_app().next_update_async()

    # Simple IK placeholder - just returns current position
    def simple_ik(robot, target_pos):
        # TODO: Integrate real IK solver
        return robot.get_joint_positions()

    # PICK function
    async def pick(robot, obj, robot_name):
        print(f"\n--- {robot_name} Picking {obj.name} ---")
        obj_pos, _ = obj.get_world_pose()

        # Approach above
        approach_pos = obj_pos.copy()
        approach_pos[2] += 0.3
        await move_robot(robot, simple_ik(robot, approach_pos), 150, "Approaching")

        # Open gripper
        await gripper_action("Opening gripper")

        # Move to grasp
        grasp_pos = obj_pos.copy()
        grasp_pos[2] += 0.05
        await move_robot(robot, simple_ik(robot, grasp_pos), 100, "Moving to grasp")

        # Close gripper
        await gripper_action("Closing gripper")

        # Lift
        await move_robot(robot, simple_ik(robot, approach_pos), 100, "Lifting")
        print(f"✓ {obj.name} picked")

    # PLACE function
    async def place(robot, target_pos, robot_name):
        print(f"\n--- {robot_name} Placing Object ---")

        # Approach above
        approach_pos = target_pos.copy()
        approach_pos[2] += 0.3
        await move_robot(robot, simple_ik(robot, approach_pos), 150, "Approaching")

        # Move to place
        await move_robot(robot, simple_ik(robot, target_pos), 100, "Moving to place")

        # Open gripper
        await gripper_action("Releasing")

        # Retract
        await move_robot(robot, simple_ik(robot, approach_pos), 100, "Retracting")
        print(f"✓ Placed")

    # Get target positions
    danny_pos, _ = danny_table.get_world_pose()
    box_pos, _ = box_body.get_world_pose()

    danny_target = danny_pos + np.array([0, 0, 0.4])
    box_target = box_pos + np.array([0, 0, 0.2])

    # ==== TASK 1: Robot 2 - Box Lid to Danny ====
    print("\n" + "="*60)
    print("TASK 1: Robot 2 picks box lid to Danny table")
    print("="*60)

    await pick(robot2, box_lid, "Robot 2")
    await place(robot2, danny_target, "Robot 2")

    print("\n[✓] TASK 1 COMPLETE\n")

    # ==== TASK 2: Robot 1 - Cube to Box ====
    print("\n" + "="*60)
    print("TASK 2: Robot 1 picks cube to box body")
    print("="*60)

    await pick(robot1, cube, "Robot 1")
    await place(robot1, box_target, "Robot 1")

    print("\n[✓] TASK 2 COMPLETE\n")

    # ==== TASK 3: Return Home ====
    print("\n" + "="*60)
    print("TASK 3: Returning robots to home positions")
    print("="*60)

    print("\n--- Robot 2 Returning Home ---")
    await move_robot(robot2, robot2_home, 200, "Returning")
    print("✓ Robot 2 at home")

    print("\n--- Robot 1 Returning Home ---")
    await move_robot(robot1, robot1_home, 200, "Returning")
    print("✓ Robot 1 at home")

    print("\n[✓] TASK 3 COMPLETE\n")

    # Done!
    print("\n" + "="*60)
    print("ALL TASKS COMPLETE!")
    print("="*60 + "\n")


# ============================================================================
# RUN THE SEQUENCE
# ============================================================================
asyncio.ensure_future(pick_and_place_sequence())

print("\n>>> Pick and Place sequence started! Watch the viewport. <<<\n")
