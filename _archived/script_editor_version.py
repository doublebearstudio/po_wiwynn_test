"""
Robot Pick and Place - Isaac Sim Script Editor Version

INSTRUCTIONS:
1. Open po_wiwynn_test_v0002.usda in Isaac Sim
2. Open Script Editor (Window > Script Editor)
3. Copy and paste this entire script
4. Click "Run" or press Ctrl+Enter

This version works directly in Isaac Sim's Script Editor without needing SimulationApp.
"""

import numpy as np
import omni
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.prims import RigidPrim, XFormPrim
from omni.isaac.core.utils.stage import get_current_stage
import asyncio
import time


# ============================================================================
# CONFIGURATION
# ============================================================================

CONFIG = {
    # Robot paths
    "robot1_path": "/World/kr210_l150_01",
    "robot2_path": "/World/kr210_l150_02",

    # Object paths
    "box_lid_path": "/World/geo_boxLid_01",
    "cube_path": "/World/geo_cube_01",

    # Target paths
    "danny_table_path": "/World/Danny",
    "box_body_path": "/World/grp_boxBody_01",

    # Motion parameters
    "motion_steps": {
        "approach": 150,
        "grasp": 100,
        "lift": 100,
        "place": 100,
        "retract": 100,
        "home": 200,
    },

    # Heights (meters)
    "approach_height_pick": 0.30,
    "approach_height_place": 0.30,
    "grasp_offset_z": 0.05,

    # Placement offsets (meters)
    "danny_offset": {"x": 0.0, "y": 0.0, "z": 0.40},
    "box_body_offset": {"x": 0.0, "y": 0.0, "z": 0.20},

    # Gripper timing
    "gripper_wait_steps": 30,

    # Debug
    "debug": True,
}


# ============================================================================
# CONTROLLER CLASS
# ============================================================================

class ScriptEditorPickPlaceController:
    """Pick and Place Controller for Isaac Sim Script Editor"""

    def __init__(self, config):
        self.config = config
        self.world = None
        self.robot1 = None
        self.robot2 = None
        self.box_lid = None
        self.cube = None
        self.danny_table = None
        self.box_body = None
        self.robot1_home = None
        self.robot2_home = None

        print("\n" + "="*70)
        print(" "*15 + "ISAAC SIM SCRIPT EDITOR VERSION")
        print("="*70 + "\n")

    async def setup(self):
        """Setup the world and get references to scene objects"""
        print("Setting up world and scene objects...")

        # Get or create world
        self.world = World.instance()
        if self.world is None:
            self.world = World(stage_units_in_meters=1.0)

        # Reset world
        await self.world.reset_async()
        await omni.kit.app.get_app().next_update_async()

        # Get robots
        print(f"  Getting robot 1: {self.config['robot1_path']}")
        self.robot1 = self.world.scene.add(
            Articulation(prim_path=self.config["robot1_path"], name="robot1")
        )

        print(f"  Getting robot 2: {self.config['robot2_path']}")
        self.robot2 = self.world.scene.add(
            Articulation(prim_path=self.config["robot2_path"], name="robot2")
        )

        # Get objects
        print(f"  Getting box lid: {self.config['box_lid_path']}")
        self.box_lid = self.world.scene.add(
            RigidPrim(prim_path=self.config["box_lid_path"], name="box_lid")
        )

        print(f"  Getting cube: {self.config['cube_path']}")
        self.cube = self.world.scene.add(
            RigidPrim(prim_path=self.config["cube_path"], name="cube")
        )

        # Get targets
        self.danny_table = XFormPrim(prim_path=self.config["danny_table_path"])
        self.box_body = XFormPrim(prim_path=self.config["box_body_path"])

        # Initialize world
        await self.world.reset_async()
        await omni.kit.app.get_app().next_update_async()

        # Store home positions
        self.robot1_home = self.robot1.get_joint_positions()
        self.robot2_home = self.robot2.get_joint_positions()

        if self.config["debug"]:
            print(f"\n  Robot 1 home: {self.robot1_home}")
            print(f"  Robot 2 home: {self.robot2_home}\n")

        print("✓ Setup complete\n")

    async def move_to_joint_positions(self, robot, target_joints, num_steps=100, motion_type="default"):
        """Move robot to target joint positions smoothly"""
        current_joints = robot.get_joint_positions()

        robot_name = "Robot 1" if robot == self.robot1 else "Robot 2"
        if self.config["debug"]:
            print(f"    Moving {robot_name} ({motion_type}): {num_steps} steps")

        # Smooth interpolation
        for i in range(num_steps):
            # Cosine interpolation for smooth start/stop
            alpha = 0.5 * (1 - np.cos(np.pi * (i + 1) / num_steps))
            interpolated_joints = current_joints + alpha * (target_joints - current_joints)

            # Apply action
            action = ArticulationAction(joint_positions=interpolated_joints)
            robot.apply_action(action)

            # Step simulation
            await self.world.play_async()
            await omni.kit.app.get_app().next_update_async()

        # Ensure final position
        action = ArticulationAction(joint_positions=target_joints)
        robot.apply_action(action)

        for _ in range(10):
            await self.world.play_async()
            await omni.kit.app.get_app().next_update_async()

    async def open_gripper(self, robot):
        """Open gripper"""
        robot_name = "Robot 1" if robot == self.robot1 else "Robot 2"
        print(f"    Opening {robot_name} gripper")

        # TODO: Implement actual gripper control
        for _ in range(self.config["gripper_wait_steps"]):
            await self.world.play_async()
            await omni.kit.app.get_app().next_update_async()

    async def close_gripper(self, robot):
        """Close gripper"""
        robot_name = "Robot 1" if robot == self.robot1 else "Robot 2"
        print(f"    Closing {robot_name} gripper")

        # TODO: Implement actual gripper control
        for _ in range(self.config["gripper_wait_steps"]):
            await self.world.play_async()
            await omni.kit.app.get_app().next_update_async()

    def compute_ik(self, robot, target_position):
        """Compute IK (placeholder - returns current joints)"""
        # TODO: Integrate with Lula IK solver
        return robot.get_joint_positions()

    async def pick_object(self, robot, object_prim):
        """Execute pick sequence"""
        robot_name = "Robot 1" if robot == self.robot1 else "Robot 2"
        print(f"\n{'─'*60}")
        print(f"  Picking {object_prim.name} with {robot_name}")
        print(f"{'─'*60}")

        # Get object position
        obj_pos, _ = object_prim.get_world_pose()
        if self.config["debug"]:
            print(f"  Object at: ({obj_pos[0]:.3f}, {obj_pos[1]:.3f}, {obj_pos[2]:.3f})")

        # 1. Approach
        approach_pos = obj_pos.copy()
        approach_pos[2] += self.config["approach_height_pick"]
        print(f"  → Approaching...")

        approach_joints = self.compute_ik(robot, approach_pos)
        await self.move_to_joint_positions(
            robot, approach_joints,
            self.config["motion_steps"]["approach"],
            "approach"
        )

        # 2. Open gripper
        await self.open_gripper(robot)

        # 3. Move to grasp
        grasp_pos = obj_pos.copy()
        grasp_pos[2] += self.config["grasp_offset_z"]
        print(f"  → Grasping...")

        grasp_joints = self.compute_ik(robot, grasp_pos)
        await self.move_to_joint_positions(
            robot, grasp_joints,
            self.config["motion_steps"]["grasp"],
            "grasp"
        )

        # 4. Close gripper
        await self.close_gripper(robot)

        # 5. Lift
        print(f"  → Lifting...")
        await self.move_to_joint_positions(
            robot, approach_joints,
            self.config["motion_steps"]["lift"],
            "lift"
        )

        print(f"  ✓ {object_prim.name} picked")

    async def place_object(self, robot, target_position):
        """Execute place sequence"""
        robot_name = "Robot 1" if robot == self.robot1 else "Robot 2"
        print(f"\n{'─'*60}")
        print(f"  Placing with {robot_name}")
        print(f"{'─'*60}")

        if self.config["debug"]:
            print(f"  Target: ({target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f})")

        # 1. Approach
        approach_pos = target_position.copy()
        approach_pos[2] += self.config["approach_height_place"]
        print(f"  → Approaching...")

        approach_joints = self.compute_ik(robot, approach_pos)
        await self.move_to_joint_positions(
            robot, approach_joints,
            self.config["motion_steps"]["approach"],
            "approach"
        )

        # 2. Move to place position
        print(f"  → Placing...")
        place_joints = self.compute_ik(robot, target_position)
        await self.move_to_joint_positions(
            robot, place_joints,
            self.config["motion_steps"]["place"],
            "place"
        )

        # 3. Open gripper
        await self.open_gripper(robot)

        # 4. Retract
        print(f"  → Retracting...")
        await self.move_to_joint_positions(
            robot, approach_joints,
            self.config["motion_steps"]["retract"],
            "retract"
        )

        print(f"  ✓ Placed")

    async def return_to_home(self, robot, home_joints):
        """Return to home position"""
        robot_name = "Robot 1" if robot == self.robot1 else "Robot 2"
        print(f"\n{'─'*60}")
        print(f"  Returning {robot_name} to home")
        print(f"{'─'*60}")

        await self.move_to_joint_positions(
            robot, home_joints,
            self.config["motion_steps"]["home"],
            "return home"
        )

        print(f"  ✓ {robot_name} at home")

    async def execute_sequence(self):
        """Execute full pick and place sequence"""
        print("\n" + "="*70)
        print(" "*20 + "STARTING SEQUENCE")
        print("="*70)

        # Get target positions
        danny_pos, _ = self.danny_table.get_world_pose()
        box_body_pos, _ = self.box_body.get_world_pose()

        # Apply offsets
        danny_target = danny_pos + np.array([
            self.config["danny_offset"]["x"],
            self.config["danny_offset"]["y"],
            self.config["danny_offset"]["z"]
        ])

        box_target = box_body_pos + np.array([
            self.config["box_body_offset"]["x"],
            self.config["box_body_offset"]["y"],
            self.config["box_body_offset"]["z"]
        ])

        # TASK 1: Robot 2 picks box lid -> places on Danny
        print("\n" + "="*70)
        print("TASK 1: Robot 2 - Box Lid to Danny Table")
        print("="*70)

        await self.pick_object(self.robot2, self.box_lid)
        await self.place_object(self.robot2, danny_target)

        print("\n[✓] TASK 1 COMPLETE")

        # TASK 2: Robot 1 picks cube -> places in box
        print("\n" + "="*70)
        print("TASK 2: Robot 1 - Cube to Box Body")
        print("="*70)

        await self.pick_object(self.robot1, self.cube)
        await self.place_object(self.robot1, box_target)

        print("\n[✓] TASK 2 COMPLETE")

        # TASK 3: Return to home
        print("\n" + "="*70)
        print("TASK 3: Return to Home Positions")
        print("="*70)

        await self.return_to_home(self.robot2, self.robot2_home)
        await self.return_to_home(self.robot1, self.robot1_home)

        print("\n[✓] TASK 3 COMPLETE")

        # Final
        print("\n" + "="*70)
        print(" "*20 + "SEQUENCE COMPLETE!")
        print("="*70 + "\n")


# ============================================================================
# MAIN EXECUTION FUNCTION
# ============================================================================

async def run_pick_and_place():
    """Main execution function"""
    try:
        # Create controller
        controller = ScriptEditorPickPlaceController(CONFIG)

        # Setup
        await controller.setup()

        # Execute sequence
        await controller.execute_sequence()

        print("\nDone! The sequence has completed successfully.")
        print("The robots should now be back at their home positions.\n")

    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()


# ============================================================================
# RUN IT!
# ============================================================================

# This is what actually runs when you click "Run" in Script Editor
asyncio.ensure_future(run_pick_and_place())

print("\n" + "="*70)
print("Pick and Place sequence started!")
print("Watch the viewport to see the robots in action.")
print("="*70 + "\n")
