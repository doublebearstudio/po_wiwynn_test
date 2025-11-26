from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.core.utils.stage import add_reference_to_stage
from isaacsim.core.api.tasks import BaseTask
from isaacsim.core.utils.prims import create_prim, get_prim_at_path
from isaacsim.core.utils.stage import get_current_stage
from typing import Optional
import numpy as np
from pxr import UsdGeom, Gf, UsdPhysics, PhysxSchema, Usd


class CustomPickPlace(BaseTask):
    """Custom pick and place task with support for different object types including custom USD files."""

    def __init__(
        self,
        name: str = "custom_pick_place",
        object_type: str = "cylinder",
        object_initial_position: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
        custom_usd_path: Optional[str] = None,
    ) -> None:
        """
        Initialize the custom pick and place task.

        Args:
            name: Task name
            object_type: Type of object to pick - "cylinder", "sphere", "cube", or "usd"
            object_initial_position: Initial position of the object [x, y, z]
            target_position: Where to place the object [x, y, z]
            offset: Offset for task positioning
            custom_usd_path: Path to custom USD file (only used when object_type="usd")
                            Can be a local path or Omniverse URL
        """
        super().__init__(name=name, offset=offset)

        self.object_type = object_type
        self._object_initial_position = object_initial_position
        self._target_position = target_position
        self._custom_usd_path = custom_usd_path
        self._robot_name = "cobotta_robot"
        self._object_name = "pickup_object"
        self._target_name = "target_position"

        # Set default positions if not provided
        if self._object_initial_position is None:
            self._object_initial_position = np.array([0.3, 0.3, 0.05])

        if self._target_position is None:
            self._target_position = np.array([-0.3, 0.3, 0.05])

        # Validate custom_usd_path if object_type is "usd"
        if self.object_type == "usd" and self._custom_usd_path is None:
            raise ValueError("custom_usd_path must be provided when object_type='usd'")

        return

    def set_robot(self) -> SingleManipulator:
        """Set up the Denso Cobotta robot with gripper."""
        asset_path = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots/Denso/cobotta_pro_900.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/cobotta")

        gripper = ParallelGripper(
            end_effector_prim_path="/World/cobotta/onrobot_rg6_base_link",
            joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],
            joint_opened_positions=np.array([0, 0]),
            joint_closed_positions=np.array([0.628, -0.628]),
            action_deltas=np.array([-0.2, 0.2])
        )

        manipulator = SingleManipulator(
            prim_path="/World/cobotta",
            name=self._robot_name,
            end_effector_prim_name="onrobot_rg6_base_link",
            gripper=gripper
        )

        joints_default_positions = np.zeros(12)
        joints_default_positions[7] = 0.628
        joints_default_positions[8] = 0.628
        manipulator.set_joints_default_state(positions=joints_default_positions)

        return manipulator

    def set_up_scene(self, scene) -> None:
        """Set up the scene with robot and custom pickup object."""
        # Store scene reference for later use
        self._scene = scene

        # Add robot
        robot = self.set_robot()
        scene.add(robot)

        # Create pickup object based on type
        if self.object_type == "usd":
            # For USD objects, just create in stage, don't add to scene
            self._create_usd_object()
        else:
            raise ValueError(f"Unknown object_type: {self.object_type}. Currently only 'usd' is supported")

        # Create target position visualizer (simple visual cube)
        self._create_target_visualizer()

        return

    # def _create_cylinder(self) -> DynamicCylinder:
    #     """Create a cylinder as the pickup object."""
    #     prim_path = f"/World/{self._object_name}"
    #
    #     # Create a dynamic cylinder with physics enabled
    #     cylinder = DynamicCylinder(
    #         prim_path=prim_path,
    #         name=self._object_name,
    #         position=self._object_initial_position,
    #         radius=0.025,  # 2.5 cm
    #         height=0.05,   # 5 cm
    #         color=np.array([0.0, 0.5, 1.0]),  # Blue
    #         mass=0.1  # 100 grams
    #     )
    #
    #     return cylinder

    # def _create_sphere(self) -> DynamicSphere:
    #     """Create a sphere as the pickup object."""
    #     prim_path = f"/World/{self._object_name}"
    #
    #     # Create a dynamic sphere with physics enabled
    #     sphere = DynamicSphere(
    #         prim_path=prim_path,
    #         name=self._object_name,
    #         position=self._object_initial_position,
    #         radius=0.03,  # 3 cm
    #         color=np.array([1.0, 0.5, 0.0]),  # Orange
    #         mass=0.05  # 50 grams
    #     )
    #
    #     return sphere

    # def _create_cube(self) -> DynamicCuboid:
    #     """Create a cube as the pickup object."""
    #     prim_path = f"/World/{self._object_name}"
    #
    #     # Create a dynamic cuboid (cube) with physics enabled
    #     size = 0.0515  # 5.15 cm
    #     cube = DynamicCuboid(
    #         prim_path=prim_path,
    #         name=self._object_name,
    #         position=self._object_initial_position,
    #         size=size,  # Creates a cube with this edge length
    #         color=np.array([1.0, 0.0, 0.0]),  # Red
    #         mass=0.1  # 100 grams
    #     )
    #
    #     return cube

    def _create_usd_object(self) -> None:
        """Load a USD file as the pickup object."""
        prim_path = f"/World/{self._object_name}"

        # Use the custom USD path provided in __init__
        add_reference_to_stage(usd_path=self._custom_usd_path, prim_path=prim_path)

        # Get the prim and set its position
        stage = get_current_stage()
        prim = stage.GetPrimAtPath(prim_path)

        # Set position
        xformable = UsdGeom.Xformable(prim)
        xform_api = UsdGeom.XformCommonAPI(xformable)
        xform_api.SetTranslate(Gf.Vec3d(
            self._object_initial_position[0],
            self._object_initial_position[1],
            self._object_initial_position[2]
        ))

        # Add physics properties if they don't already exist
        # This ensures the custom USD object can be picked up
        if not prim.HasAPI(UsdPhysics.RigidBodyAPI):
            UsdPhysics.RigidBodyAPI.Apply(prim)

        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdPhysics.CollisionAPI.Apply(prim)

        if not prim.HasAPI(PhysxSchema.PhysxCollisionAPI):
            PhysxSchema.PhysxCollisionAPI.Apply(prim)

        # Set a default mass if not already set (100 grams)
        mass_api = UsdPhysics.MassAPI.Apply(prim)
        if not mass_api.GetMassAttr().Get():
            mass_api.GetMassAttr().Set(0.01)

        return

    def _create_target_visualizer(self) -> None:
        """Create a visual marker for the target position (green platform)."""
        prim_path = "/World/target_position_viz"
        stage = get_current_stage()

        # Create a thin cube as visual marker
        cube_geom = UsdGeom.Cube.Define(stage, prim_path)

        # Set size (5cm x 5cm x 2mm platform)
        cube_geom.GetSizeAttr().Set(0.05)

        # Set color (green)
        cube_geom.CreateDisplayColorAttr().Set([Gf.Vec3f(0.0, 1.0, 0.0)])

        # Position it at target location
        xform_api = UsdGeom.XformCommonAPI(cube_geom)
        xform_api.SetTranslate(Gf.Vec3d(
            self._target_position[0],
            self._target_position[1],
            0.001  # Just above ground
        ))
        xform_api.SetScale(Gf.Vec3f(1.0, 1.0, 0.04))  # Make it thin (platform-like)

        # No physics - just visual
        return

    def get_observations(self) -> dict:
        """Get current observations for the task."""
        # Get robot object
        robot = self._scene.get_object(self._robot_name)

        if robot is None:
            raise RuntimeError(f"Robot '{self._robot_name}' not found in scene")

        # Get pickup object position from USD stage
        stage = get_current_stage()
        object_prim = stage.GetPrimAtPath(f"/World/{self._object_name}")

        if not object_prim.IsValid():
            raise RuntimeError(f"Object '{self._object_name}' prim not found at /World/{self._object_name}")

        xformable = UsdGeom.Xformable(object_prim)

        # Get world transform at current time
        time_code = Usd.TimeCode.Default()
        world_transform = xformable.ComputeLocalToWorldTransform(time_code)
        translation = world_transform.ExtractTranslation()
        position = np.array([translation[0], translation[1], translation[2]])

        # Get orientation (rotation as quaternion)
        rotation = world_transform.ExtractRotationQuat()
        orientation = np.array([rotation.GetImaginary()[0], rotation.GetImaginary()[1],
                               rotation.GetImaginary()[2], rotation.GetReal()])

        observations = {
            self._robot_name: {
                "joint_positions": robot.get_joint_positions(),
            },
            self._object_name: {
                "position": position,
                "orientation": orientation,
                "target_position": self._target_position,
            }
        }

        return observations

    def get_params(self) -> dict:
        """Get task parameters."""
        params = {
            "robot_name": {"value": self._robot_name},
            "cube_name": {"value": self._object_name},  # Keep "cube_name" for compatibility with controller
            "target_name": {"value": self._target_name},
            "object_type": {"value": self.object_type},
        }
        return params

    def pre_step(self, time_step_index: int, simulation_time: float) -> None:
        """Called before each simulation step."""
        return

    def post_reset(self) -> None:
        """Called after the task is reset."""
        return
