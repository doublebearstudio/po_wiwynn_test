"""
Simple Pick and Place Scene Setup (No BaseTask inheritance)

This module provides a straightforward way to set up a pick-and-place scene
without the complexity of the BaseTask framework. Perfect for Script Editor
and extension development.

Usage:
    from simple_pick_place_setup import PickPlaceSceneSetup

    setup = PickPlaceSceneSetup(
        custom_usd_path="path/to/object.usd",
        object_initial_position=np.array([-0.5, 0.4, 0.125]),
        target_position=np.array([-0.6, -0.5, 0.125])
    )

    # Setup the scene
    robot = setup.create_robot()
    setup.create_object()
    setup.create_target_marker()

    # Get data for control loop
    object_pos = setup.get_object_position()
    target_pos = setup.target_position
"""

from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from typing import Optional, Tuple
import numpy as np
from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema, Usd


class PickPlaceSceneSetup:
    """Simple scene setup for pick-and-place tasks without BaseTask framework."""

    def __init__(
        self,
        custom_usd_path: str,
        object_initial_position: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        robot_prim_path: str = "/World/cobotta",
        object_prim_path: str = "/World/pickup_object",
    ):
        """
        Initialize the pick-and-place scene setup.

        Args:
            custom_usd_path: Path to custom USD file for the object to pick up
            object_initial_position: Initial position [x, y, z] in meters
            target_position: Target position [x, y, z] in meters
            robot_prim_path: USD path for the robot (default: /World/cobotta)
            object_prim_path: USD path for the object (default: /World/pickup_object)
        """
        self.custom_usd_path = custom_usd_path
        self.object_initial_position = object_initial_position if object_initial_position is not None else np.array([0.3, 0.3, 0.05])
        self.target_position = target_position if target_position is not None else np.array([-0.3, 0.3, 0.05])

        self.robot_prim_path = robot_prim_path
        self.object_prim_path = object_prim_path
        self.target_marker_path = "/World/target_marker"

        self._robot = None
        self._stage = get_current_stage()

    def create_robot(self) -> SingleManipulator:
        """
        Create and setup the Denso Cobotta robot with OnRobot RG6 gripper.

        Returns:
            SingleManipulator: The configured robot manipulator
        """
        # Load robot USD
        robot_asset = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots/Denso/cobotta_pro_900.usd"
        add_reference_to_stage(usd_path=robot_asset, prim_path=self.robot_prim_path)

        # Configure gripper
        gripper = ParallelGripper(
            end_effector_prim_path=f"{self.robot_prim_path}/onrobot_rg6_base_link",
            joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],
            joint_opened_positions=np.array([0, 0]),
            joint_closed_positions=np.array([0.628, -0.628]),
            action_deltas=np.array([-0.2, 0.2])
        )

        # Create manipulator
        manipulator = SingleManipulator(
            prim_path=self.robot_prim_path,
            name="cobotta_robot",
            end_effector_prim_name="onrobot_rg6_base_link",
            gripper=gripper
        )

        # Set default joint positions (gripper open)
        joints_default_positions = np.zeros(12)
        joints_default_positions[7] = 0.628
        joints_default_positions[8] = 0.628
        manipulator.set_joints_default_state(positions=joints_default_positions)

        self._robot = manipulator
        return manipulator

    def create_object(self, mass: float = 0.01) -> str:
        """
        Create the pickup object from USD file.

        Args:
            mass: Mass in kg (default: 0.01 kg = 10 grams)

        Returns:
            str: Prim path of the created object
        """
        # Load custom USD
        add_reference_to_stage(usd_path=self.custom_usd_path, prim_path=self.object_prim_path)

        # Get the prim
        prim = self._stage.GetPrimAtPath(self.object_prim_path)

        # Set position
        xformable = UsdGeom.Xformable(prim)
        xform_api = UsdGeom.XformCommonAPI(xformable)
        xform_api.SetTranslate(Gf.Vec3d(
            float(self.object_initial_position[0]),
            float(self.object_initial_position[1]),
            float(self.object_initial_position[2])
        ))

        # # Add physics properties
        # if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
        #     UsdPhysics.RigidBodyAPI.Apply(prim)
        #
        # if not prim.HasAPI(UsdPhysics.CollisionAPI):
        #     UsdPhysics.CollisionAPI.Apply(prim)
        #
        # if not prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
        #     physx_collision = PhysxSchema.PhysxCollisionAPI.Apply(prim)
        #     # Use convex hull for dynamic objects
        #     physx_collision.CreateApproximationAttr().Set("convexHull")

        # Set mass
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        mass_api.GetMassAttr().Set(mass)

        return self.object_prim_path

    def create_target_marker(self, size: float = 0.05, height: float = 0.002) -> str:
        """
        Create a visual marker at the target position.

        Args:
            size: Marker size in meters (default: 5cm)
            height: Marker height/thickness (default: 2mm)

        Returns:
            str: Prim path of the marker
        """
        # Create a thin cube as visual marker
        cube_geom = UsdGeom.Cube.Define(self._stage, self.target_marker_path)

        # Set size
        cube_geom.GetSizeAttr().Set(size)

        # Set color (green)
        cube_geom.CreateDisplayColorAttr().Set([Gf.Vec3f(0.0, 1.0, 0.0)])

        # Position at target
        xform_api = UsdGeom.XformCommonAPI(cube_geom)
        xform_api.SetTranslate(Gf.Vec3d(
            float(self.target_position[0]),
            float(self.target_position[1]),
            0.001  # Just above ground
        ))
        xform_api.SetScale(Gf.Vec3f(1.0, 1.0, height / size))  # Make thin

        return self.target_marker_path

    def get_object_position(self) -> np.ndarray:
        """
        Get current position of the pickup object.

        Returns:
            np.ndarray: Position [x, y, z] in meters
        """
        prim = self._stage.GetPrimAtPath(self.object_prim_path)
        if not prim.IsValid():
            raise RuntimeError(f"Object prim not found at {self.object_prim_path}")

        xformable = UsdGeom.Xformable(prim)
        world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        translation = world_transform.ExtractTranslation()

        return np.array([translation[0], translation[1], translation[2]])

    def get_object_orientation(self) -> np.ndarray:
        """
        Get current orientation of the pickup object.

        Returns:
            np.ndarray: Quaternion [x, y, z, w]
        """
        prim = self._stage.GetPrimAtPath(self.object_prim_path)
        if not prim.IsValid():
            raise RuntimeError(f"Object prim not found at {self.object_prim_path}")

        xformable = UsdGeom.Xformable(prim)
        world_transform = xformable.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        rotation = world_transform.ExtractRotationQuat()

        # Return as [x, y, z, w]
        return np.array([
            rotation.GetImaginary()[0],
            rotation.GetImaginary()[1],
            rotation.GetImaginary()[2],
            rotation.GetReal()
        ])

    def get_observations(self, robot) -> dict:
        """
        Get observations in the format expected by PickPlaceController.

        Args:
            robot: The robot manipulator object

        Returns:
            dict: Observations dictionary
        """
        return {
            "cobotta_robot": {
                "joint_positions": robot.get_joint_positions(),
            },
            "pickup_object": {
                "position": self.get_object_position(),
                "orientation": self.get_object_orientation(),
                "target_position": self.target_position,
            }
        }

    @property
    def robot(self) -> Optional[SingleManipulator]:
        """Get the robot manipulator (if created)."""
        return self._robot
