from isaacsim.robot.manipulators.manipulators import SingleManipulator
from isaacsim.robot.manipulators.grippers import ParallelGripper
from isaacsim.core.utils.stage import add_reference_to_stage
import isaacsim.core.api.tasks as tasks
from typing import Optional
import numpy as np
from follow_target import SingleManipulatorNoGripper

class PickPlace(tasks.PickPlace):
    def __init__(
        self,
        name: str = "kuka_pick_place",
        cube_initial_position: Optional[np.ndarray] = None,
        cube_initial_orientation: Optional[np.ndarray] = None,
        target_position: Optional[np.ndarray] = None,
        offset: Optional[np.ndarray] = None,
    ) -> None:
        tasks.PickPlace.__init__(
            self,
            name=name,
            cube_initial_position=cube_initial_position,
            cube_initial_orientation=cube_initial_orientation,
            target_position=target_position,
            cube_size=np.array([0.0515, 0.0515, 0.0515]),
            offset=offset,
        )
        return

    def set_robot(self) -> SingleManipulator:
        asset_path = "https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/4.5/Isaac/Robots/Kuka/KR210_L150/kr210_l150.usd"
        add_reference_to_stage(usd_path=asset_path, prim_path="/World/kuka_kr210")
        # gripper = ParallelGripper(
        #     end_effector_prim_path="/World/kuka_kr210/tool0",
        #     joint_prim_names=["finger_joint", "right_outer_knuckle_joint"],
        #     joint_opened_positions=np.array([0, 0]),
        #     joint_closed_positions=np.array([0.628, -0.628]),
        #     action_deltas=np.array([-0.2, 0.2]) )
        manipulator = SingleManipulatorNoGripper(prim_path="/World/kuka_kr210",
                                        name="kuka_robot",
                                        end_effector_prim_name="tool0")
        joints_default_positions = np.zeros(6)
        # joints_default_positions[7] = 0.628
        # joints_default_positions[8] = 0.628
        manipulator.set_joints_default_state(positions=joints_default_positions)
        return manipulator