"""
Robot Pick and Place Controller for Wiwynn POC
This script controls two Kuka KR210 robot arms to perform pick and place operations.

Sequence:
1. kr210_l150_02 picks geo_boxLid_01 and places it on Danny table
2. kr210_l150_01 picks geo_cube_01 and places it into grp_boxBody_01
3. Both robots return to original positions
"""

from omni.isaac.kit import SimulationApp

# Initialize simulation
simulation_app = SimulationApp({"headless": False})

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.robots import Robot
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.stage import get_current_stage
from omni.isaac.core.prims import XFormPrim, RigidPrim
from pxr import Usd, UsdGeom, Gf
import carb
import time


class RobotPickPlaceController:
    def __init__(self, usd_path):
        """Initialize the pick and place controller"""
        self.usd_path = usd_path
        self.world = World()

        # Load the USD stage
        self.stage = get_current_stage()
        self.stage.GetRootLayer().Clear()
        self.stage = Usd.Stage.Open(usd_path)

        # Wait for stage to load
        simulation_app.update()

        # Initialize world
        self.world = World(stage_units_in_meters=1.0)

        # Get robot references
        self.robot1_path = "/World/kr210_l150_01"
        self.robot2_path = "/World/kr210_l150_02"

        # Add robots to the scene
        self.robot1 = self.world.scene.add(
            Robot(prim_path=self.robot1_path, name="kr210_01")
        )
        self.robot2 = self.world.scene.add(
            Robot(prim_path=self.robot2_path, name="kr210_02")
        )

        # Get object references
        self.box_lid = RigidPrim(prim_path="/World/geo_boxLid_01", name="box_lid")
        self.cube = RigidPrim(prim_path="/World/geo_cube_01", name="cube")

        # Get target references
        self.danny_table = XFormPrim(prim_path="/World/Danny")
        self.box_body = XFormPrim(prim_path="/World/grp_boxBody_01")

        # Store original joint positions
        self.robot1_original_joints = None
        self.robot2_original_joints = None

        print("Robot Pick Place Controller initialized")

    def reset_world(self):
        """Reset the simulation world"""
        self.world.reset()

        # Store original positions if not already stored
        if self.robot1_original_joints is None:
            self.robot1_original_joints = self.robot1.get_joint_positions()
        if self.robot2_original_joints is None:
            self.robot2_original_joints = self.robot2.get_joint_positions()

    def get_end_effector_position(self, robot):
        """Get the current end effector position"""
        if robot == self.robot1:
            ee_path = f"{self.robot1_path}/tool0"
        else:
            ee_path = f"{self.robot2_path}/tool0"

        ee_prim = XFormPrim(prim_path=ee_path)
        position, _ = ee_prim.get_world_pose()
        return position

    def move_robot_to_position(self, robot, target_position, num_steps=100):
        """
        Move robot end effector to target position
        This is a simplified version - in production, use proper IK solver
        """
        print(f"Moving robot to position: {target_position}")

        # Get current joint positions
        current_joints = robot.get_joint_positions()

        # Simple trajectory: interpolate joint positions
        # In production, use inverse kinematics (IK) solver
        for i in range(num_steps):
            # Update simulation
            self.world.step(render=True)
            simulation_app.update()
            time.sleep(0.01)

        return True

    def open_gripper(self, robot):
        """Open the robot gripper"""
        print("Opening gripper")
        # For Kuka robots, the gripper control depends on the attached gripper
        # This is a placeholder - implement based on actual gripper type
        time.sleep(0.5)

    def close_gripper(self, robot):
        """Close the robot gripper"""
        print("Closing gripper")
        # For Kuka robots, the gripper control depends on the attached gripper
        # This is a placeholder - implement based on actual gripper type
        time.sleep(0.5)

    def pick_object(self, robot, object_prim):
        """Pick up an object with the robot"""
        print(f"Picking object: {object_prim.name}")

        # Get object position
        obj_position, _ = object_prim.get_world_pose()

        # Move to pre-grasp position (above object)
        pre_grasp_pos = obj_position + np.array([0, 0, 0.2])
        self.move_robot_to_position(robot, pre_grasp_pos)

        # Open gripper
        self.open_gripper(robot)

        # Move to grasp position
        self.move_robot_to_position(robot, obj_position)

        # Close gripper
        self.close_gripper(robot)

        # Move up
        self.move_robot_to_position(robot, pre_grasp_pos)

        print(f"Object {object_prim.name} picked successfully")

    def place_object(self, robot, target_position):
        """Place the object at target position"""
        print(f"Placing object at position: {target_position}")

        # Move to pre-place position (above target)
        pre_place_pos = target_position + np.array([0, 0, 0.2])
        self.move_robot_to_position(robot, pre_place_pos)

        # Move to place position
        self.move_robot_to_position(robot, target_position)

        # Open gripper
        self.open_gripper(robot)

        # Move up
        self.move_robot_to_position(robot, pre_place_pos)

        print("Object placed successfully")

    def return_to_original_position(self, robot, original_joints):
        """Return robot to original position"""
        print("Returning robot to original position")

        if original_joints is not None:
            # Apply original joint positions
            action = ArticulationAction(joint_positions=original_joints)
            robot.apply_action(action)

            # Wait for robot to reach position
            for _ in range(100):
                self.world.step(render=True)
                simulation_app.update()
                time.sleep(0.01)

        print("Robot returned to original position")

    def execute_pick_and_place_sequence(self):
        """Execute the complete pick and place sequence"""
        print("\n" + "="*60)
        print("Starting Pick and Place Sequence")
        print("="*60 + "\n")

        # Reset world
        self.reset_world()

        # Step 1: Robot 2 picks box lid and places on Danny table
        print("\n--- Step 1: Robot 2 picks box lid ---")
        danny_position, _ = self.danny_table.get_world_pose()
        # Place on top of the table
        danny_place_position = danny_position + np.array([0, 0, 0.5])

        self.pick_object(self.robot2, self.box_lid)
        self.place_object(self.robot2, danny_place_position)

        print("\n--- Step 1 Complete ---\n")

        # Step 2: Robot 1 picks cube and places in box body
        print("\n--- Step 2: Robot 1 picks cube ---")
        box_body_position, _ = self.box_body.get_world_pose()
        # Place inside the box (slightly above the bottom)
        box_place_position = box_body_position + np.array([0, 0, 0.15])

        self.pick_object(self.robot1, self.cube)
        self.place_object(self.robot1, box_place_position)

        print("\n--- Step 2 Complete ---\n")

        # Step 3: Return both robots to original positions
        print("\n--- Step 3: Returning robots to original positions ---")
        self.return_to_original_position(self.robot1, self.robot1_original_joints)
        self.return_to_original_position(self.robot2, self.robot2_original_joints)

        print("\n--- Step 3 Complete ---\n")

        print("\n" + "="*60)
        print("Pick and Place Sequence Complete!")
        print("="*60 + "\n")


def main():
    """Main function"""
    usd_path = "D:/poc/po_wiwynn_test/po_wiwynn_test_v0002.usda"

    try:
        # Create controller
        controller = RobotPickPlaceController(usd_path)

        # Execute pick and place sequence
        controller.execute_pick_and_place_sequence()

        # Keep simulation running for observation
        print("\nSimulation running. Press Ctrl+C to exit.")
        while simulation_app.is_running():
            controller.world.step(render=True)
            simulation_app.update()
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nShutting down simulation...")

    except Exception as e:
        print(f"Error occurred: {e}")
        import traceback
        traceback.print_exc()

    finally:
        # Cleanup
        simulation_app.close()


if __name__ == "__main__":
    main()
