from abc import ABC

from cfrankr import Robot, MotionData, Affine

from frankpiv.backend.general import GeneralBackend


class Backend(GeneralBackend, ABC):

    def __init__(self, config):
        super().__init__(config)
        self.frankr_config = config["frankr"]
        self._robot = None
        self._motion_data = None

    def _initialize(self):
        self._robot = Robot(self.frankr_config["robot_name"], self.frankr_config["dynamic_rel"])
        self._motion_data = MotionData()

    def _finish(self):
        self._robot = None
        self._motion_data = None

    def _current_pose(self) -> Affine:
        return self._robot.current_pose(Affine())

    def _move_robot_cartesian(self, target_pose: Affine):
        self._robot.move_cartesian(Affine(), target_pose, self._motion_data)
