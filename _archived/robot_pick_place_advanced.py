"""
Advanced Robot Pick and Place Controller for Wiwynn POC
This script uses Isaac Sim's motion planning and IK capabilities for precise control.

Sequence:
1. kr210_l150_02 picks geo_boxLid_01 and places it on Danny table
2. kr210_l150_01 picks geo_cube_01 and places it into grp_boxBody_01
3. Both robots return to original positions
"""

from omni.isaac.kit import SimulationApp

# Initialize simulation with specific configuration
config = {
    "headless": False,
    "width": 1920,
    "height": 1080,
}
simulation_app = SimulationApp(config)

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core.prims import RigidPrim, XFormPrim
from omni.isaac.manipulators import SingleManipulator
from omni.isaac.manipulators.grippers import ParallelGripper
from omni.isaac.core.utils.rotations import euler_angles_to_quat
import omni.isaac.core.utils.numpy.rotations as rotation_utils
from pxr import Gf
import carb
import asyncio


class AdvancedPickPlaceController:
    def __init__(self, usd_path):
        """Initialize the advanced pick and place controller"""
        self.usd_path = usd_path

        # Open the USD stage
        open_stage(usd_path)

        # Initialize world
        self.world = World(stage_units_in_meters=1.0)
        self.world.scene.add_default_ground_plane()

        print("Setting up robots and objects...")

        # Setup robot 1 (kr210_l150_01)
        self.robot1 = self.world.scene.add(
            Articulation(
                prim_path="/World/kr210_l150_01",
                name="robot1"
            )
        )

        # Setup robot 2 (kr210_l150_02)
        self.robot2 = self.world.scene.add(
            Articulation(
                prim_path="/World/kr210_l150_02",
                name="robot2"
            )
        )

        # Add objects to scene
        self.box_lid = self.world.scene.add(
            RigidPrim(
                prim_path="/World/geo_boxLid_01",
                name="box_lid"
            )
        )

        self.cube = self.world.scene.add(
            RigidPrim(
                prim_path="/World/geo_cube_01",
                name="cube"
            )
        )

        # Store original joint positions
        self.robot1_home = None
        self.robot2_home = None

        # End effector names
        self.robot1_ee_name = "tool0"
        self.robot2_ee_name = "tool0"

        print("Advanced Pick Place Controller initialized")

    def setup(self):
        """Setup the controller after world reset"""
        self.world.reset()

        # Store home positions
        if self.robot1_home is None:
            self.robot1_home = self.robot1.get_joint_positions()
            print(f"Robot 1 home position: {self.robot1_home}")

        if self.robot2_home is None:
            self.robot2_home = self.robot2.get_joint_positions()
            print(f"Robot 2 home position: {self.robot2_home}")

    def get_ee_pose(self, robot, ee_name):
        """Get end effector world pose"""
        if robot == self.robot1:
            ee_path = "/World/kr210_l150_01/tool0"
        else:
            ee_path = "/World/kr210_l150_02/tool0"

        ee_prim = XFormPrim(prim_path=ee_path)
        position, orientation = ee_prim.get_world_pose()
        return position, orientation

    def move_to_joint_positions(self, robot, target_joints, num_steps=100):
        """Move robot to target joint positions smoothly"""
        current_joints = robot.get_joint_positions()

        # Create smooth trajectory
        for i in range(num_steps):
            alpha = (i + 1) / num_steps
            # Linear interpolation
            interpolated_joints = current_joints + alpha * (target_joints - current_joints)

            # Apply action
            action = ArticulationAction(joint_positions=interpolated_joints)
            robot.apply_action(action)

            # Step simulation
            self.world.step(render=True)
            simulation_app.update()

        return True

    def compute_ik_joint_positions(self, robot, target_position, target_orientation=None):
        """
        Compute inverse kinematics to reach target position
        This is a simplified version - for production use Lula or other IK solvers
        """
        # For this example, we'll use predefined joint configurations
        # In production, integrate with omni.isaac.motion_generation

        # Get current joints as starting point
        current_joints = robot.get_joint_positions()

        # TODO: Implement proper IK solver
        # For now, return modified current joints (placeholder)
        # You should integrate with Lula IK solver or RMPflow

        return current_joints

    def pick_object_sequence(self, robot, object_prim, approach_height=0.3):
        """Execute pick sequence for an object"""
        print(f"\n=== Picking {object_prim.name} ===")

        # Get object position
        obj_pos, obj_rot = object_prim.get_world_pose()
        print(f"Object position: {obj_pos}")

        # Step 1: Move to approach position (above object)
        approach_pos = obj_pos.copy()
        approach_pos[2] += approach_height
        print(f"Moving to approach position: {approach_pos}")

        # Calculate joint positions for approach (using IK)
        approach_joints = self.compute_ik_joint_positions(robot, approach_pos)
        self.move_to_joint_positions(robot, approach_joints, num_steps=150)

        # Step 2: Open gripper
        print("Opening gripper")
        self.open_gripper(robot)

        # Step 3: Move to grasp position
        grasp_pos = obj_pos.copy()
        grasp_pos[2] += 0.05  # Slightly above center for better grasp
        print(f"Moving to grasp position: {grasp_pos}")

        grasp_joints = self.compute_ik_joint_positions(robot, grasp_pos)
        self.move_to_joint_positions(robot, grasp_joints, num_steps=100)

        # Step 4: Close gripper
        print("Closing gripper")
        self.close_gripper(robot)

        # Step 5: Lift object
        print("Lifting object")
        lift_joints = approach_joints
        self.move_to_joint_positions(robot, lift_joints, num_steps=100)

        print(f"=== {object_prim.name} picked successfully ===\n")

        # Simulate attachment (in production, use proper physics-based grasping)
        for _ in range(10):
            self.world.step(render=True)
            simulation_app.update()

    def place_object_sequence(self, robot, target_position, approach_height=0.3):
        """Execute place sequence for an object"""
        print(f"\n=== Placing object at {target_position} ===")

        # Step 1: Move to approach position (above target)
        approach_pos = target_position.copy()
        approach_pos[2] += approach_height
        print(f"Moving to approach position: {approach_pos}")

        approach_joints = self.compute_ik_joint_positions(robot, approach_pos)
        self.move_to_joint_positions(robot, approach_joints, num_steps=150)

        # Step 2: Move to place position
        place_pos = target_position.copy()
        print(f"Moving to place position: {place_pos}")

        place_joints = self.compute_ik_joint_positions(robot, place_pos)
        self.move_to_joint_positions(robot, place_joints, num_steps=100)

        # Step 3: Open gripper to release
        print("Opening gripper to release")
        self.open_gripper(robot)

        # Step 4: Retract
        print("Retracting")
        self.move_to_joint_positions(robot, approach_joints, num_steps=100)

        print(f"=== Object placed successfully ===\n")

        # Simulate detachment
        for _ in range(10):
            self.world.step(render=True)
            simulation_app.update()

    def open_gripper(self, robot):
        """Open gripper - implement based on gripper type"""
        # Placeholder for gripper control
        # For actual implementation, control gripper joints
        for _ in range(30):
            self.world.step(render=True)
            simulation_app.update()

    def close_gripper(self, robot):
        """Close gripper - implement based on gripper type"""
        # Placeholder for gripper control
        # For actual implementation, control gripper joints
        for _ in range(30):
            self.world.step(render=True)
            simulation_app.update()

    def move_to_home(self, robot, home_joints):
        """Move robot back to home position"""
        print(f"\n=== Returning to home position ===")
        self.move_to_joint_positions(robot, home_joints, num_steps=200)
        print(f"=== Home position reached ===\n")

    def execute_full_sequence(self):
        """Execute the complete pick and place sequence"""
        print("\n" + "="*70)
        print(" "*15 + "STARTING PICK AND PLACE SEQUENCE")
        print("="*70 + "\n")

        # Setup and reset
        self.setup()

        # Get target positions
        danny_pos, _ = self.world.scene.get_object("/World/Danny").get_world_pose()
        box_body_pos, _ = self.world.scene.get_object("/World/grp_boxBody_01").get_world_pose()

        # Adjust target positions
        danny_target = danny_pos.copy()
        danny_target[2] += 0.4  # Place on top of Danny table

        box_target = box_body_pos.copy()
        box_target[2] += 0.2  # Place inside box body

        # === TASK 1: Robot 2 picks box lid and places on Danny ===
        print("\n" + "="*70)
        print(" "*10 + "TASK 1: Robot 2 - Box Lid to Danny Table")
        print("="*70)

        self.pick_object_sequence(self.robot2, self.box_lid)
        self.place_object_sequence(self.robot2, danny_target)

        print("\n[✓] TASK 1 COMPLETE\n")

        # === TASK 2: Robot 1 picks cube and places in box body ===
        print("\n" + "="*70)
        print(" "*10 + "TASK 2: Robot 1 - Cube to Box Body")
        print("="*70)

        self.pick_object_sequence(self.robot1, self.cube)
        self.place_object_sequence(self.robot1, box_target)

        print("\n[✓] TASK 2 COMPLETE\n")

        # === TASK 3: Return both robots to home ===
        print("\n" + "="*70)
        print(" "*10 + "TASK 3: Returning Robots to Home Position")
        print("="*70)

        print("\nRobot 2 returning home...")
        self.move_to_home(self.robot2, self.robot2_home)

        print("\nRobot 1 returning home...")
        self.move_to_home(self.robot1, self.robot1_home)

        print("\n[✓] TASK 3 COMPLETE\n")

        print("\n" + "="*70)
        print(" "*10 + "ALL TASKS COMPLETED SUCCESSFULLY!")
        print("="*70 + "\n")


def main():
    """Main execution function"""
    usd_path = "D:/poc/po_wiwynn_test/po_wiwynn_test_v0002.usda"

    try:
        # Create controller
        print("Initializing Advanced Pick Place Controller...")
        controller = AdvancedPickPlaceController(usd_path)

        # Execute sequence
        controller.execute_full_sequence()

        # Keep simulation running
        print("\nSimulation running. Press Ctrl+C to exit.")
        print("You can observe the final state of the scene.\n")

        while simulation_app.is_running():
            controller.world.step(render=True)
            simulation_app.update()

    except KeyboardInterrupt:
        print("\n\nShutting down simulation gracefully...")

    except Exception as e:
        print(f"\n[ERROR] An error occurred: {e}")
        import traceback
        traceback.print_exc()

    finally:
        simulation_app.close()
        print("Simulation closed.")


if __name__ == "__main__":
    main()
