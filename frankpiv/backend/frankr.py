import sys
from abc import ABC
from enum import IntEnum

import numpy as np
import rospy
from cfrankr import Robot, MotionData, Affine
from geometry_msgs.msg import Vector3, Pose, Point
from moveit_commander import roscpp_initialize, roscpp_shutdown, MoveGroupCommander
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker

from frankpiv.backend.general import GeneralBackend


class Backend(GeneralBackend, ABC):
    class MarkerType(IntEnum):
        Axis = 5
        Point = 2

    def __init__(self, config):
        super().__init__(config)
        self.frankr_config = config["frankr"]
        self.eef_ppoint_distance = config["eef_ppoint_distance"]
        self.tool_length = config["tool_length"]
        self._robot = None
        self._move_group = None
        self._motion_data = None
        self._reference_frame = None
        # visualization
        self._visualize = self.frankr_config["rviz_marker"] if "rviz_marker" in self.frankr_config else False
        self._marker_publisher = None

    def _publish_marker(self, pose: Affine, id: int = 0, type: MarkerType = MarkerType.Axis):
        header = Header(frame_id="world", stamp=rospy.Time.now())
        root = Point(*pose.to_array()[:3])

        if type == Backend.MarkerType.Axis:
            point_x = Point(*(pose * Affine(x=0.05)).to_array()[:3])
            point_y = Point(*(pose * Affine(y=0.05)).to_array()[:3])
            point_z = Point(*(pose * Affine(z=0.05)).to_array()[:3])
            marker = Marker(header=header, id=id, type=type, action=0, pose=Pose(),
                            scale=Vector3(x=0.01), points=[root, point_x, root, point_y, root, point_z],
                            colors=[ColorRGBA(1, 0, 0, 1), ColorRGBA(1, 0, 0, 1),
                                    ColorRGBA(0, 1, 0, 1), ColorRGBA(0, 1, 0, 1),
                                    ColorRGBA(0, 0, 1, 1), ColorRGBA(0, 0, 1, 1)])
        elif type == Backend.MarkerType.Point:
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

    def start(self):
        roscpp_initialize(sys.argv)
        rospy.init_node("pivot_controller", anonymous=True)
        self._robot = Robot(self.frankr_config["robot_name"], self.frankr_config["dynamic_rel"])
        self._move_group = MoveGroupCommander(self.frankr_config["robot_name"])
        self._motion_data = MotionData()
        home = [0.0, -np.pi / 4, 0.0, -3 * np.pi / 4, 0.0, np.pi / 2, np.pi / 4]
        self._robot.move_joints(home, self._motion_data)
        self._reference_frame = self._robot.current_pose(Affine()) * Affine(z=self.eef_ppoint_distance)
        if self._visualize:
            self._marker_publisher = rospy.Publisher("visualization_marker", Marker, queue_size=10)
            rospy.sleep(1)
            self._publish_marker(self._reference_frame, id=0)

    def stop(self):
        self._robot = None
        self._move_group = None
        self._motion_data = None
        if self._visualize:
            self._delete_marker()
            self._marker_publisher = None
        rospy.signal_shutdown("done")
        roscpp_shutdown()

    def move_to_point(self, frame, point):
        raise NotImplementedError()

    def move_pyrz(self, pjrz):
        pitch, yaw, roll, z_translation = pjrz
        target_affine = Affine(*self._reference_frame.to_array())
        target_affine *= Affine(b=yaw, c=pitch) * Affine(a=roll)
        target_affine.translate([0, 0, z_translation - self.eef_ppoint_distance])
        if self._visualize:
            self._publish_marker(target_affine, id=1)
        self._robot.move_cartesian(Affine(), target_affine, self._motion_data)
        if self._visualize:
            self._delete_marker(id=1)

