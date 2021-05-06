import sys
import threading
from abc import ABC

import moveit_commander
from cfrankr import Affine

from frankpiv.backend.conversions import pose_msg_to_affine, affine_to_pose_msg
from frankpiv.backend.general import GeneralBackend


class Backend(GeneralBackend, ABC):

    def __init__(self, config, async_motion=False):
        super().__init__(config)
        self.moveit_config = config["moveit"]
        self._robot = None
        self._motion_data = None
        self.async_motion = async_motion
        self._thread_queue = []
        self._thread_queue_lock = threading.Lock()
        self._has_stopped = threading.Event()
        self._has_stopped.set()

    def _initialize(self):
        self._init_ros_node()
        moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.MoveGroupCommander(self.moveit_config["robot_name"])

    def _finish(self):
        self._has_stopped.wait()
        moveit_commander.roscpp_shutdown()
        self._shutdown_ros_node()
        self._robot = None

    def _current_pose(self) -> Affine:
        return pose_msg_to_affine(self._robot.get_current_pose())

    def _move_robot_cartesian(self, target_pose: Affine):
        with self._thread_queue_lock:
            is_most_recent = self._thread_queue.index(threading.current_thread().ident) == (len(self._thread_queue) - 1)
        if is_most_recent:
            self._robot.go(affine_to_pose_msg(target_pose))

    def _move_pyrz_internal(self, pjrz, degrees):
        self._has_stopped.clear()
        with self._thread_queue_lock:
            self._thread_queue.append(threading.current_thread().ident)
        self._robot.stop()
        super().move_pyrz(pjrz, degrees)
        with self._thread_queue_lock:
            self._thread_queue.remove(threading.current_thread().ident)
            if not self._thread_queue:
                self._has_stopped.set()

    def move_pyrz(self, pjrz, degrees=False):
        if self.async_motion:
            threading.Thread(target=Backend._move_pyrz_internal, args=(self, pjrz, degrees)).start()
        else:
            self._move_pyrz_internal(pjrz, degrees)
