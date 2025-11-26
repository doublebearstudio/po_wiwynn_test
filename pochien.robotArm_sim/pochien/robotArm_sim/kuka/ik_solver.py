from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from isaacsim.core.prims import Articulation
from typing import Optional


class KinematicsSolver(ArticulationKinematicsSolver):
    def __init__(self, robot_articulation: Articulation, end_effector_frame_name: Optional[str] = None) -> None:
        #TODO: change the config path
        self._kinematics = LulaKinematicsSolver(robot_description_path="D:/poc/po_wiwynn_test/pochien.robotArm_sim/pochien/robotArm_sim/robot_descriptor.yaml",
                                                urdf_path="D:/poc/po_wiwynn_test/pochien.robotArm_sim/pochien/robotArm_sim/kr210_l150.urdf")
        if end_effector_frame_name is None:
            end_effector_frame_name = "tool0"
        ArticulationKinematicsSolver.__init__(self, robot_articulation, self._kinematics, end_effector_frame_name)
        return