import sys
from abc import ABC

import moveit_commander
from cfrankr import Affine

from frankpiv.backend.conversions import pose_msg_to_affine, affine_to_pose_msg
from frankpiv.backend.general import GeneralBackend


class Backend(GeneralBackend, ABC):

    def __init__(self, config):
        super().__init__(config)
        self.moveit_config = config["moveit"]
        self._robot = None
        self._motion_data = None

    def _initialize(self):
        self._init_ros_node()
        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.MoveGroupCommander(self.moveit_config["robot_name"])

    def _finish(self):
        moveit_commander.roscpp_shutdown()
        self._shutdown_ros_node()
        self._robot = None

    def _current_pose(self) -> Affine:
        return pose_msg_to_affine(self._robot.get_current_pose())

    def _move_robot_cartesian(self, target_pose: Affine):
        self._robot.go(affine_to_pose_msg(target_pose))
