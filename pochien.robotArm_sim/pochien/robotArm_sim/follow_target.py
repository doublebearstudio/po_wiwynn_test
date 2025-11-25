from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.core.utils.stage import add_reference_to_stage
import isaacsim.core.api.tasks as tasks
from typing import Optional
import numpy as np


# Custom SingleManipulator class that handles robots without grippers
class SingleManipulatorNoGripper(SingleManipulator):
    def post_reset(self) -> None:
        # Override post_reset to handle None gripper case
        super(SingleManipulator, self).post_reset()
        if self._gripper is not None:
            self._gripper.post_reset()
        return


# Inheriting from the base class Follow Target
class FollowTarget(tasks.FollowTarget):
    def __init__(
        self,
        name: str = "kuka_follow_target",
        target_prim_path: Optional[str] = None,
        target_name: Optional[str] = None,
        target_position: Optional[np.ndarray] = None,
        target_orientation: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        tasks.FollowTarget.__init__(
            self,
            name=name,
            target_prim_path=target_prim_path,
            target_name=target_name,
            target_position=target_position,
            target_orientation=target_orientation,
            offset=offset,
        )
        return

    def set_robot(self) -> SingleManipulator:
        #TODO: change this to the robot USD file.
        asset_path = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots/Kuka/KR210_L150/kr210_l150.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/kuka_kr210")
        # KUKA KR210 does not have a gripper, so no gripper configuration needed
        manipulator = SingleManipulatorNoGripper(prim_path="/World/kuka_kr210",
                                                  name="kuka_robot",
                                                  end_effector_prim_name="tool0")
        # KUKA KR210 has 6 joints
        joints_default_positions = np.zeros(6)
        manipulator.set_joints_default_state(positions=joints_default_positions)
        return manipulator
