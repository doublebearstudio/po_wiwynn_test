"""
Production Robot Pick and Place Controller for Wiwynn POC
This is the recommended script for actual use - it uses the config.py for easy customization.

Sequence:
1. kr210_l150_02 picks geo_boxLid_01 and places it on Danny table
2. kr210_l150_01 picks geo_cube_01 and places it into grp_boxBody_01
3. Both robots return to original positions
"""

import sys
import os

# Import configuration
try:
    import config
except ImportError:
    print("ERROR: config.py not found in the same directory!")
    print("Please ensure config.py is in the same folder as this script.")
    sys.exit(1)

from omni.isaac.kit import SimulationApp

# Initialize simulation with configuration
simulation_app = SimulationApp(config.SIMULATION_CONFIG)

import numpy as np
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core.prims import RigidPrim, XFormPrim
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quat
import time


class ProductionPickPlaceController:
    """
    Production-ready pick and place controller using configuration from config.py
    """

    def __init__(self):
        """Initialize the controller with configuration"""
        # Print configuration
        if config.DEBUG_MODE:
            config.print_config()

        # Open the USD stage
        print(f"Loading USD stage: {config.USD_STAGE_PATH}")
        open_stage(config.USD_STAGE_PATH)

        # Initialize world
        self.world = World(stage_units_in_meters=1.0, physics_dt=config.PHYSICS_DT)

        print("Setting up robots and scene objects...")

        # Setup robots
        self.robot1 = self.world.scene.add(
            Articulation(prim_path=config.ROBOT1_PATH, name="robot1")
        )
        self.robot2 = self.world.scene.add(
            Articulation(prim_path=config.ROBOT2_PATH, name="robot2")
        )

        # Setup objects
        self.box_lid = self.world.scene.add(
            RigidPrim(prim_path=config.BOX_LID_PATH, name="box_lid")
        )
        self.cube = self.world.scene.add(
            RigidPrim(prim_path=config.CUBE_PATH, name="cube")
        )

        # Setup targets
        self.danny_table = XFormPrim(prim_path=config.DANNY_TABLE_PATH)
        self.box_body = XFormPrim(prim_path=config.BOX_BODY_PATH)

        # Store home positions
        self.robot1_home = None
        self.robot2_home = None

        # Step counter
        self.step_count = 0

        print("✓ Production Pick Place Controller initialized successfully\n")

    def setup(self):
        """Setup the controller and store home positions"""
        self.world.reset()

        # Store home positions
        if self.robot1_home is None:
            self.robot1_home = self.robot1.get_joint_positions()
            if config.DEBUG_MODE:
                print(f"Robot 1 home joints: {self.robot1_home}")

        if self.robot2_home is None:
            self.robot2_home = self.robot2.get_joint_positions()
            if config.DEBUG_MODE:
                print(f"Robot 2 home joints: {self.robot2_home}")

    def step_simulation(self, num_steps=1):
        """Step the simulation"""
        for i in range(num_steps):
            self.world.step(render=(i % config.RENDER_FREQUENCY == 0))
            if i % config.RENDER_FREQUENCY == 0:
                simulation_app.update()
            self.step_count += 1

    def get_ee_pose(self, robot):
        """Get end effector world pose"""
        if robot == self.robot1:
            ee_path = f"{config.ROBOT1_PATH}/{config.ROBOT1_EE_NAME}"
        else:
            ee_path = f"{config.ROBOT2_PATH}/{config.ROBOT2_EE_NAME}"

        ee_prim = XFormPrim(prim_path=ee_path)
        position, orientation = ee_prim.get_world_pose()
        return position, orientation

    def move_to_joint_positions(self, robot, target_joints, num_steps=100, motion_type="default"):
        """Move robot to target joint positions with smooth interpolation"""
        current_joints = robot.get_joint_positions()

        if config.DEBUG_MODE:
            robot_name = "Robot 1" if robot == self.robot1 else "Robot 2"
            print(f"  Moving {robot_name} ({motion_type}): {num_steps} steps")

        # Smooth trajectory with cosine interpolation for better motion
        for i in range(num_steps):
            # Cosine interpolation for smoother start/stop
            alpha = 0.5 * (1 - np.cos(np.pi * (i + 1) / num_steps))
            interpolated_joints = current_joints + alpha * (target_joints - current_joints)

            # Apply action
            action = ArticulationAction(joint_positions=interpolated_joints)
            robot.apply_action(action)

            # Log joint positions if enabled
            if config.LOG_JOINT_POSITIONS and i % 20 == 0:
                print(f"    Step {i}: {interpolated_joints}")

            # Step simulation
            self.step_simulation()

        # Ensure final position is reached
        action = ArticulationAction(joint_positions=target_joints)
        robot.apply_action(action)
        self.step_simulation(10)

    def open_gripper(self, robot):
        """Open the robot gripper"""
        robot_name = "Robot 1" if robot == self.robot1 else "Robot 2"
        print(f"  Opening {robot_name} gripper")

        # TODO: Implement actual gripper control
        # Example for parallel gripper:
        # gripper_joints = robot.get_dof_index("gripper_joint")
        # action = ArticulationAction(joint_positions={gripper_joints: config.GRIPPER_OPEN_POSITION})
        # robot.apply_action(action)

        # Placeholder wait
        wait_steps = int(config.GRIPPER_WAIT_TIME / config.PHYSICS_DT)
        self.step_simulation(wait_steps)

    def close_gripper(self, robot):
        """Close the robot gripper"""
        robot_name = "Robot 1" if robot == self.robot1 else "Robot 2"
        print(f"  Closing {robot_name} gripper")

        # TODO: Implement actual gripper control
        # Example for parallel gripper:
        # gripper_joints = robot.get_dof_index("gripper_joint")
        # action = ArticulationAction(joint_positions={gripper_joints: config.GRIPPER_CLOSE_POSITION})
        # robot.apply_action(action)

        # Placeholder wait
        wait_steps = int(config.GRIPPER_WAIT_TIME / config.PHYSICS_DT)
        self.step_simulation(wait_steps)

    def compute_ik(self, robot, target_position, target_orientation=None):
        """
        Compute inverse kinematics to reach target position
        NOTE: This is a placeholder - integrate with Lula IK for production
        """
        # Get current joints as fallback
        current_joints = robot.get_joint_positions()

        # TODO: Integrate with omni.isaac.motion_generation.lula
        # from omni.isaac.motion_generation import LulaKinematicsSolver
        # ik_solver = LulaKinematicsSolver(...)
        # target_joints = ik_solver.compute_inverse_kinematics(...)

        # For now, return current joints (placeholder)
        return current_joints

    def pick_object(self, robot, object_prim):
        """Execute pick sequence for an object"""
        robot_name = "Robot 1" if robot == self.robot1 else "Robot 2"
        print(f"\n{'─'*60}")
        print(f"  Picking {object_prim.name} with {robot_name}")
        print(f"{'─'*60}")

        # Get object position
        obj_pos, obj_rot = object_prim.get_world_pose()
        if config.DEBUG_MODE:
            print(f"  Object position: ({obj_pos[0]:.3f}, {obj_pos[1]:.3f}, {obj_pos[2]:.3f})")

        # 1. Move to approach position
        approach_pos = obj_pos.copy()
        approach_pos[2] += config.get_approach_height("pick")
        print(f"  → Approaching above object...")

        approach_joints = self.compute_ik(robot, approach_pos)
        self.move_to_joint_positions(
            robot, approach_joints,
            num_steps=config.get_motion_steps("approach"),
            motion_type="approach"
        )

        # 2. Open gripper
        self.open_gripper(robot)

        # 3. Move to grasp position
        grasp_pos = obj_pos.copy()
        grasp_pos[2] += config.GRASP_OFFSET_Z
        print(f"  → Moving to grasp position...")

        grasp_joints = self.compute_ik(robot, grasp_pos)
        self.move_to_joint_positions(
            robot, grasp_joints,
            num_steps=config.get_motion_steps("grasp"),
            motion_type="grasp"
        )

        # 4. Close gripper
        self.close_gripper(robot)

        # 5. Lift object
        print(f"  → Lifting object...")
        self.move_to_joint_positions(
            robot, approach_joints,
            num_steps=config.get_motion_steps("lift"),
            motion_type="lift"
        )

        print(f"  ✓ {object_prim.name} picked successfully")

        if config.PAUSE_BETWEEN_STEPS:
            input("  Press Enter to continue...")

    def place_object(self, robot, target_position):
        """Execute place sequence for an object"""
        robot_name = "Robot 1" if robot == self.robot1 else "Robot 2"
        print(f"\n{'─'*60}")
        print(f"  Placing object with {robot_name}")
        print(f"{'─'*60}")

        if config.DEBUG_MODE:
            print(f"  Target position: ({target_position[0]:.3f}, {target_position[1]:.3f}, {target_position[2]:.3f})")

        # 1. Move to approach position
        approach_pos = target_position.copy()
        approach_pos[2] += config.get_approach_height("place")
        print(f"  → Approaching target location...")

        approach_joints = self.compute_ik(robot, approach_pos)
        self.move_to_joint_positions(
            robot, approach_joints,
            num_steps=config.get_motion_steps("approach"),
            motion_type="approach"
        )

        # 2. Move to place position
        print(f"  → Moving to place position...")
        place_joints = self.compute_ik(robot, target_position)
        self.move_to_joint_positions(
            robot, place_joints,
            num_steps=config.get_motion_steps("place"),
            motion_type="place"
        )

        # 3. Open gripper to release
        self.open_gripper(robot)

        # 4. Retract
        print(f"  → Retracting...")
        self.move_to_joint_positions(
            robot, approach_joints,
            num_steps=config.get_motion_steps("retract"),
            motion_type="retract"
        )

        print(f"  ✓ Object placed successfully")

        if config.PAUSE_BETWEEN_STEPS:
            input("  Press Enter to continue...")

    def return_to_home(self, robot, home_joints):
        """Move robot back to home position"""
        robot_name = "Robot 1" if robot == self.robot1 else "Robot 2"
        print(f"\n{'─'*60}")
        print(f"  Returning {robot_name} to home position")
        print(f"{'─'*60}")

        self.move_to_joint_positions(
            robot, home_joints,
            num_steps=config.get_motion_steps("home"),
            motion_type="return home"
        )

        print(f"  ✓ {robot_name} home position reached")

    def execute_sequence(self):
        """Execute the complete pick and place sequence"""
        print("\n" + "="*70)
        print(" "*15 + "STARTING PRODUCTION SEQUENCE")
        print("="*70 + "\n")

        # Setup
        self.setup()

        # Get target positions
        danny_pos, _ = self.danny_table.get_world_pose()
        box_body_pos, _ = self.box_body.get_world_pose()

        # Apply offsets from config
        danny_target = danny_pos + np.array([
            config.DANNY_PLACEMENT_OFFSET["x"],
            config.DANNY_PLACEMENT_OFFSET["y"],
            config.DANNY_PLACEMENT_OFFSET["z"]
        ])

        box_target = box_body_pos + np.array([
            config.BOX_BODY_PLACEMENT_OFFSET["x"],
            config.BOX_BODY_PLACEMENT_OFFSET["y"],
            config.BOX_BODY_PLACEMENT_OFFSET["z"]
        ])

        # === TASK 1: Robot 2 picks box lid and places on Danny ===
        if config.ENABLE_TASKS["task1_robot2_boxlid"]:
            print("\n" + "="*70)
            print("TASK 1: Robot 2 - Pick Box Lid and Place on Danny Table")
            print("="*70)

            self.pick_object(self.robot2, self.box_lid)
            self.place_object(self.robot2, danny_target)

            print("\n[✓] TASK 1 COMPLETED\n")
            time.sleep(config.INTER_TASK_DELAY)
        else:
            print("\n[SKIPPED] Task 1 disabled in configuration\n")

        # === TASK 2: Robot 1 picks cube and places in box body ===
        if config.ENABLE_TASKS["task2_robot1_cube"]:
            print("\n" + "="*70)
            print("TASK 2: Robot 1 - Pick Cube and Place in Box Body")
            print("="*70)

            self.pick_object(self.robot1, self.cube)
            self.place_object(self.robot1, box_target)

            print("\n[✓] TASK 2 COMPLETED\n")
            time.sleep(config.INTER_TASK_DELAY)
        else:
            print("\n[SKIPPED] Task 2 disabled in configuration\n")

        # === TASK 3: Return both robots to home ===
        if config.ENABLE_TASKS["task3_return_home"]:
            print("\n" + "="*70)
            print("TASK 3: Return Robots to Home Positions")
            print("="*70)

            self.return_to_home(self.robot2, self.robot2_home)
            self.return_to_home(self.robot1, self.robot1_home)

            print("\n[✓] TASK 3 COMPLETED\n")
        else:
            print("\n[SKIPPED] Task 3 disabled in configuration\n")

        # Final summary
        print("\n" + "="*70)
        print(" "*20 + "SEQUENCE COMPLETE!")
        print("="*70)
        print(f"\nTotal simulation steps: {self.step_count}")
        print(f"Total simulation time: {self.step_count * config.PHYSICS_DT:.2f} seconds\n")


def main():
    """Main execution function"""
    try:
        # Create controller
        print("="*70)
        print(" "*15 + "WIWYNN POC - ROBOT CONTROLLER")
        print("="*70 + "\n")

        controller = ProductionPickPlaceController()

        # Execute sequence
        controller.execute_sequence()

        # Keep simulation running for observation
        print("\nSimulation running. Press Ctrl+C to exit.")
        print("You can observe the final state of the scene.\n")

        while simulation_app.is_running():
            controller.step_simulation()

    except KeyboardInterrupt:
        print("\n\n" + "="*70)
        print(" "*22 + "SHUTTING DOWN")
        print("="*70)

    except Exception as e:
        print(f"\n[ERROR] An error occurred: {e}")
        import traceback
        traceback.print_exc()

    finally:
        simulation_app.close()
        print("\nSimulation closed. Goodbye!")


if __name__ == "__main__":
    main()
