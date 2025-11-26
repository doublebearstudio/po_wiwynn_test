"""
Pick and Place Task Definition for Denso Cobotta Robot

This module defines a custom PickPlace task that sets up the Denso Cobotta Pro 900
robot with an OnRobot RG6 gripper for pick-and-place operations in Isaac Sim.

The task inherits from Isaac Sim's base PickPlace task and customizes it by:
- Loading the Denso Cobotta robot USD model
- Configuring the OnRobot RG6 parallel gripper
- Setting up default joint positions
- Defining the cube object to be manipulated

Usage:
    from pick_place import PickPlace

    task = PickPlace(
        name="my_task",
        cube_initial_position=np.array([0.3, 0.3, 0.03]),
        target_position=np.array([-0.3, 0.6, 0.03])
    )

    world.add_task(task)

Classes:
    PickPlace: Custom pick-and-place task for Denso Cobotta robot
"""

from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.core.utils.stage import add_reference_to_stage
import isaacsim.core.api.tasks as tasks
from typing import Optional
import numpy as np


class PickPlace(tasks.PickPlace):
    """
    Pick and Place task for Denso Cobotta Pro 900 robot with OnRobot RG6 gripper.

    This class extends Isaac Sim's base PickPlace task to work with the Denso Cobotta
    robot. It handles robot loading, gripper configuration, and scene setup.

    The task creates a scene with:
    - Denso Cobotta Pro 900 robot (6-DOF arm)
    - OnRobot RG6 parallel gripper (2-finger)
    - A cube object to pick up (5.15cm default size)
    - A target position marker for placement

    Attributes:
        Robot: Denso Cobotta Pro 900
        Gripper: OnRobot RG6 (2-finger parallel)
        DOF: 6 arm joints + 6 gripper joints = 12 total
        Cube size: 5.15 cm × 5.15 cm × 5.15 cm (default)
    """
    def __init__(
        self,
        name: str = "denso_pick_place",
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        """
        Initialize the Denso Cobotta pick-and-place task.

        Sets up the task parameters including cube position, orientation, target
        location, and size. The cube defaults to 5.15 cm on each side.

        Args:
            name (str): Name of the task. Default: "denso_pick_place"
            cube_initial_position (Optional[np.ndarray]): Initial [x, y, z] position
                of the cube in meters. If None, uses parent class default
                (typically [0.3, 0.3, 0.0257]).
            cube_initial_orientation (Optional[np.ndarray]): Initial orientation of
                the cube as quaternion [x, y, z, w]. If None, uses default (no rotation).
            target_position (Optional[np.ndarray]): Target [x, y, z] position where
                the cube should be placed in meters. If None, uses parent class default.
            offset (Optional[np.ndarray]): Position offset for the entire task.
                If None, no offset is applied.

        Returns:
            None

        Example:
            >>> task = PickPlace(
            ...     name="my_task",
            ...     cube_initial_position=np.array([0.3, 0.3, 0.03]),
            ...     target_position=np.array([-0.3, 0.6, 0.03])
            ... )

        Note:
            - Cube size is fixed at 5.15 cm (0.0515 m) on all sides
            - To change cube size, modify the cube_size parameter in the parent init call
            - All positions are in world coordinates (meters)
        """
        tasks.PickPlace.__init__(
            self,
            name=name,
            cube_initial_position=cube_initial_position,
            cube_initial_orientation=cube_initial_orientation,
            target_position=target_position,
            cube_size=np.array([0.0515, 0.0515, 0.0515]),  # 5.15 cm cube
            offset=offset,
        )
        return

    def set_robot(self) -> SingleManipulator:
        """
        Load and configure the Denso Cobotta robot with OnRobot RG6 gripper.

        This method is called by the parent PickPlace task during scene setup.
        It performs the following steps:
        1. Loads the Denso Cobotta Pro 900 USD model from Omniverse
        2. Configures the OnRobot RG6 parallel gripper
        3. Creates a SingleManipulator wrapper for the robot
        4. Sets default joint positions (gripper slightly open)

        Returns:
            SingleManipulator: Configured robot manipulator object with:
                - 6 arm joints (revolute)
                - 6 gripper joints (4 active, 2 mimic)
                - OnRobot RG6 gripper attached
                - Default pose with gripper slightly open

        Robot Specifications:
            - Model: Denso Cobotta Pro 900
            - Reach: 900mm
            - Payload: 900g
            - DOF: 6 (arm)
            - End-effector: onrobot_rg6_base_link

        Gripper Specifications:
            - Model: OnRobot RG6
            - Type: 2-finger parallel
            - Stroke: 160mm (0-160mm adjustable)
            - Controlled joints:
                * finger_joint (left finger)
                * right_outer_knuckle_joint (right finger)
            - Open position: [0, 0] radians
            - Closed position: [0.628, -0.628] radians (~36 degrees)
            - Action deltas: [-0.2, 0.2] (speed of opening/closing)

        Joint Configuration (12 total):
            Indices 0-5: Arm joints (shoulder, elbow, wrist)
            Indices 6-11: Gripper joints
                - Index 6: finger_joint (left outer knuckle)
                - Index 7: left_outer_knuckle_joint (mimic)
                - Index 8: left_inner_knuckle_joint (mimic)
                - Index 9: right_outer_knuckle_joint (right outer knuckle)
                - Index 10: right_inner_knuckle_joint (mimic)
                - Index 11: right_inner_finger_joint (mimic)

        Default Joint Positions:
            - Arm joints (0-5): 0.0 radians (home position)
            - Gripper joints (6-11): Mostly 0.0, with joints 7,8 at 0.628 (slightly open)

        Scene Hierarchy:
            /World/cobotta (robot root)
                └─ onrobot_rg6_base_link (end-effector/gripper mount)
                    ├─ finger_joint (left finger)
                    └─ right_outer_knuckle_joint (right finger)

        Note:
            - The robot is loaded from Omniverse at the path specified in asset_path
            - The gripper is already embedded in the Cobotta USD model
            - Joint 7 and 8 are set to 0.628 to keep gripper slightly open by default
            - This prevents the gripper from being fully closed during initialization
        """
        # ====================================================================
        # 1. Load Robot USD Model
        # ====================================================================
        asset_path = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots/Denso/cobotta_pro_900.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/cobotta")

        # ====================================================================
        # 2. Configure OnRobot RG6 Gripper
        # ====================================================================
        gripper = ParallelGripper(
            # Path to the gripper's base link (where it attaches to robot)
            end_effector_prim_path="/World/cobotta/onrobot_rg6_base_link",

            # Names of the two controllable gripper joints
            # These are the primary joints that open/close the gripper
            joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],

            # Joint positions when gripper is fully open (radians)
            joint_opened_positions=np.array([0, 0]),

            # Joint positions when gripper is fully closed (radians)
            # ~36 degrees for each finger
            joint_closed_positions=np.array([0.628, -0.628]),

            # Speed/increment of gripper motion (radians per action)
            # Negative for left finger (closes inward)
            # Positive for right finger (closes inward)
            action_deltas=np.array([-0.2, 0.2])
        )

        # ====================================================================
        # 3. Create Robot Manipulator Wrapper
        # ====================================================================
        manipulator = SingleManipulator(
            # Path to robot root in USD scene
            prim_path="/World/cobotta",

            # Name used to reference robot in the scene
            name="cobotta_robot",

            # Name of the end-effector link (gripper mount point)
            end_effector_prim_name="onrobot_rg6_base_link",

            # Attach configured gripper
            gripper=gripper
        )

        # ====================================================================
        # 4. Set Default Joint Positions
        # ====================================================================
        # Initialize all 12 joints (6 arm + 6 gripper) to zero
        joints_default_positions = np.zeros(12)

        # Set gripper joints 7 and 8 to slightly open position
        # This prevents gripper from starting fully closed
        joints_default_positions[7] = 0.628   # left_outer_knuckle_joint
        joints_default_positions[8] = 0.628   # left_inner_knuckle_joint

        # Apply default joint state to the manipulator
        manipulator.set_joints_default_state(positions=joints_default_positions)

        return manipulator

