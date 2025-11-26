from isaacsim.robot_motion.motion_generation import ArticulationKinematicsSolver, LulaKinematicsSolver
from isaacsim.core.prims import Articulation
from typing import Optional


class KinematicsSolver(ArticulationKinematicsSolver):
    def __init__(self, robot_articulation: Articulation, end_effector_frame_name: Optional[str] = None) -> None:
        #TODO: change the config path
        self._kinematics = LulaKinematicsSolver(robot_description_path="D:/poc/po_wiwynn_test/pochien.robotArm_sim/docs/robot_desciptor.yaml",
                                                urdf_path="D:/isaac_sim_v4_5_0/extscache/isaacsim.asset.importer.urdf-2.3.10+106.4.0.wx64.r.cp310/data/urdf/robots/cobotta_pro_900/cobotta_pro_900.urdf")
        if end_effector_frame_name is None:
            end_effector_frame_name = "onrobot_rg6_base_link"
        ArticulationKinematicsSolver.__init__(self, robot_articulation, self._kinematics, end_effector_frame_name)
        return