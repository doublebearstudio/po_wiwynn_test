import isaacsim.robot_motion.motion_generation as mg
from isaacsim.core.prims import Articulation


class RMPFlowController(mg.MotionPolicyController):
    def __init__(self, name: str, robot_articulation: Articulation, physics_dt: float = 1.0 / 60.0) -> None:
        # TODO: change the follow paths
        self.rmpflow = mg.lula.motion_policies.RmpFlow(robot_description_path="D:/poc/po_wiwynn_test/pochien.robotArm_sim/pochien/robotArm_sim/robot_descriptor.yaml",
                                                        rmpflow_config_path="D:/poc/po_wiwynn_test/pochien.robotArm_sim/pochien/robotArm_sim/kuka_rmpflow_common.yaml",
                                                        urdf_path="D:/poc/po_wiwynn_test/pochien.robotArm_sim/pochien/robotArm_sim/kr210_l150.urdf",
                                                        end_effector_frame_name="tool0",
                                                        maximum_substep_size=0.00334)

        self.articulation_rmp = mg.ArticulationMotionPolicy(robot_articulation, self.rmpflow, physics_dt)

        mg.MotionPolicyController.__init__(self, name=name, articulation_motion_policy=self.articulation_rmp)
        self._default_position, self._default_orientation = (
            self._articulation_motion_policy._robot_articulation.get_world_pose()
        )
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )
        return

    def reset(self):
        mg.MotionPolicyController.reset(self)
        self._motion_policy.set_robot_base_pose(
            robot_position=self._default_position, robot_orientation=self._default_orientation
        )