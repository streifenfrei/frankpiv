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
        # configuration
        self.frankr_config = config["frankr"]
        self.inital_eef_ppoint_distance = config["eef_ppoint_distance"]
        self.tool_length = config["tool_length"]
        self.pitch_boundaries = np.radians(config["pitch_boundaries"])
        self.yaw_boundaries = np.radians(config["yaw_boundaries"])
        self.roll_boundaries = np.radians(config["roll_boundaries"])
        self.z_translation_boundaries = config["z_translation_boundaries"]
        # robotic stuff
        self._robot = None
        self._move_group = None
        self._motion_data = None
        self._reference_frame = None
        self._pyrz = None
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
        if self._robot is None:
            self._robot = Robot(self.frankr_config["robot_name"], self.frankr_config["dynamic_rel"])
        if self._move_group is None:
            self._move_group = MoveGroupCommander(self.frankr_config["robot_name"])
        self._motion_data = MotionData()
        self._reference_frame = self._robot.current_pose(Affine()) * Affine(z=self.inital_eef_ppoint_distance)
        self._pyrz = [0, 0, 0, 0]
        if self._visualize:
            if self._marker_publisher is None:
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

    def move_pyrz(self, pjrz, degrees=False):
        pitch, yaw, roll, z_translation = pjrz
        if degrees:
            pitch = np.radians(pitch)
            yaw = np.radians(yaw)
            roll = np.radians(roll)
        pitch = np.clip(pitch, *self.pitch_boundaries)
        yaw = np.clip(yaw, *self.yaw_boundaries)
        roll = np.clip(roll, *self.roll_boundaries)
        z_translation = np.clip(z_translation, *self.z_translation_boundaries)
        current_pitch, current_yaw, current_roll, current_z_translation = self._pyrz
        current_eef_ppoint_distance = self.inital_eef_ppoint_distance - current_z_translation
        new_eef_ppoint_distance = self.inital_eef_ppoint_distance - z_translation
        target_affine = self._reference_frame * Affine(b=yaw, c=pitch)
        target_affine *= Affine(z=-new_eef_ppoint_distance, a=roll)
        if self._visualize:
            self._publish_marker(target_affine, id=1)
        # if relative z-translation is negative do it before pitch and yaw
        if new_eef_ppoint_distance >= current_eef_ppoint_distance:
            intermediate_affine = self._reference_frame * Affine(b=current_yaw, c=current_pitch) * \
                                  Affine(z=-new_eef_ppoint_distance, a=roll)
            self._robot.move_cartesian(Affine(), intermediate_affine, self._motion_data)
        else:
            intermediate_affine = self._reference_frame * Affine(b=yaw, c=pitch) * \
                                  Affine(z=-current_eef_ppoint_distance, a=current_roll)
            self._robot.move_cartesian(Affine(), intermediate_affine, self._motion_data)
        self._robot.move_cartesian(Affine(), target_affine, self._motion_data)
        self._pyrz = [pitch, yaw, roll, z_translation]
        if self._visualize:
            self._delete_marker(id=1)
