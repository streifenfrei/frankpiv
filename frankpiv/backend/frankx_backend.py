from abc import ABC

from frankx import Robot, Affine, LinearMotion

from frankpiv.backend.general import GeneralBackend


class Backend(GeneralBackend, ABC):

    def __init__(self, config):
        super().__init__(config)
        self.frankx_config = config["frankx"]
        self._robot = None

    def _initialize(self):
        self._robot = Robot(self.frankx_config["robot_name"], self.frankx_config["dynamic_rel"])
        self._robot.set_dynamic_rel(self.frankx_config["dynamic_rel"])

    def _finish(self):
        self._robot = None

    # TODO fix in lab
    def _current_pose(self) -> Affine:
        return self._robot.current_pose(Affine())

    def _move_robot_cartesian(self, target_pose: Affine):
        self._robot.move_cartesian(LinearMotion(target_pose))

