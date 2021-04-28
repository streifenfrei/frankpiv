import sys
from abc import ABC, abstractmethod
from enum import IntEnum
from typing import Union

import moveit_commander
import numpy as np
import rospy
from cfrankr import Affine
from geometry_msgs.msg import Point, Vector3, Pose
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker


class GeneralBackend(ABC):

    class MarkerType(IntEnum):
        Axis = 5
        Point = 2

    def __init__(self, config: dict, node_name="pivot_controller"):
        self._visualize = config["rviz_marker"] if "rviz_marker" in config else False
        self._node_name = node_name
        self._marker_publisher = None

    def start(self):
        if self._visualize:
            moveit_commander.roscpp_initialize(sys.argv)
            rospy.init_node(self._node_name, anonymous=True)
            if self._marker_publisher is None:
                self._marker_publisher = rospy.Publisher("visualization_marker", Marker, queue_size=10)
                rospy.sleep(1)

    def stop(self):
        if self._visualize:
            self._delete_marker()
            self._marker_publisher = None
            rospy.signal_shutdown("done")
            moveit_commander.roscpp_shutdown()

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()

    @abstractmethod
    def move_to_point(self, point: Union[list, tuple, np.array], roll: float, frame: Union[list, tuple, np.array]):
        pass

    @abstractmethod
    def move_pyrz(self, pjrz: Union[list, tuple, np.array], degrees: bool):
        pass

    @abstractmethod
    def move_pyrz_relative(self, pjrz: Union[list, tuple, np.array], degrees: bool):
        pass

    def _publish_marker(self, pose: Affine, id: int = 0, type: MarkerType = MarkerType.Axis):
        header = Header(frame_id="world", stamp=rospy.Time.now())
        root = Point(*pose.to_array()[:3])
        if type == GeneralBackend.MarkerType.Axis:
            point_x = Point(*(pose * Affine(x=0.05)).to_array()[:3])
            point_y = Point(*(pose * Affine(y=0.05)).to_array()[:3])
            point_z = Point(*(pose * Affine(z=0.05)).to_array()[:3])
            marker = Marker(header=header, id=id, type=type, action=0, pose=Pose(),
                            scale=Vector3(x=0.01), points=[root, point_x, root, point_y, root, point_z],
                            colors=[ColorRGBA(1, 0, 0, 1), ColorRGBA(1, 0, 0, 1),
                                    ColorRGBA(0, 1, 0, 1), ColorRGBA(0, 1, 0, 1),
                                    ColorRGBA(0, 0, 1, 1), ColorRGBA(0, 0, 1, 1)])
        elif type == GeneralBackend.MarkerType.Point:
            marker = Marker(header=header, id=id, type=type, action=0, pose=Pose(position=root),
                            scale=Vector3(0.01, 0.01, 0.01), color=ColorRGBA(0, 0, 0, 1))
        else:
            raise ValueError(f"No such MarkerType: {type}")
        self._marker_publisher.publish(marker)
        rospy.sleep(1)

    def _delete_marker(self, id=None):
        action = 3 if id is None else 2
        self._marker_publisher.publish(Marker(id=id, action=action))
        rospy.sleep(1)


class UnattainablePoseException(Exception):
    pass
