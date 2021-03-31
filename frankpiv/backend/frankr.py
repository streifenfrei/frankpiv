import operator
import sys
from abc import ABC
from enum import IntEnum
from typing import Union

import numpy as np
import rospy
from cfrankr import Robot, MotionData, Affine
from geometry_msgs.msg import Vector3, Pose, Point
from moveit_commander import roscpp_initialize, roscpp_shutdown, MoveGroupCommander
from scipy.spatial.transform.rotation import Rotation
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker

from frankpiv.backend.general import GeneralBackend, UnattainablePoseException


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
        self.max_angle = np.radians(config["max_angle"])
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
            # self._delete_marker()
            self._marker_publisher = None
        rospy.signal_shutdown("done")
        roscpp_shutdown()

    def _clip_pose(self, pose: Affine):
        pose_local = self._reference_frame.inverse() * pose
        point = pose_local.to_array()[:3]
        if point[2] < 0:
            raise ValueError("Pose must have positive z value in the reference frame")
        z_translation = np.linalg.norm(point)
        if point[0] == point[1] == point[2] == 0.0:
            point_offset = (pose_local * Affine(z=1)).to_array()[:3]
            angle = np.inner(point_offset, [0, 0, 1])
        else:
            angle = np.inner(point, [0, 0, 1]) / z_translation
        angle = np.arccos(angle)
        if angle < -self.max_angle or angle > self.max_angle or \
                z_translation < self.z_translation_boundaries[0] or z_translation > self.z_translation_boundaries[1]:
            angle = np.clip(angle, [-self.max_angle, self.max_angle])
            z_translation = np.clip(z_translation, self.z_translation_boundaries)
            rotation_axis = np.cross(point, [0, 0, 1])
            rotation_axis = rotation_axis / np.linalg.norm(rotation_axis) * angle
            clipped_pose = self._reference_frame * \
                           Affine(0, 0, 0, *Rotation.from_rotvec(rotation_axis).as_euler("zyx")) * \
                           Affine(z=z_translation)
            return True, clipped_pose
        return False, pose

    def _pose_to_pyz(self, pose: Affine):
        pose = (self._reference_frame.inverse() * pose).to_array()
        distance = np.linalg.norm(pose[:3])
        z_translation = distance if pose[2] >= 0 else -distance
        return pose[5], pose[4], z_translation

    def move_to_point(self, point: Union[list, tuple, np.array], roll: float,
                      frame: Union[list, tuple, np.array] = None):
        point = Affine(x=point[0], y=point[1], z=point[2])
        if frame is not None:
            point = Affine(frame).inverse() * point
        point = (self._reference_frame.inverse() * point).to_array()[:3]
        if point[0] == point[1] == point[2] == 0:
            point = Affine()
        else:
            if point[2] < 0:
                raise UnattainablePoseException("Point must have positive z value in reference frame")
            distance = np.linalg.norm(point)
            z_translation = distance if point[2] >= 0 else -distance
            angle = np.inner(point, [0, 0, 1])
            rotation_axis = np.cross(point, [0, 0, 1])
            rotation_axis = rotation_axis / np.linalg.norm(rotation_axis) * angle
            point = Affine(0, 0, 0, *Rotation.from_rotvec(rotation_axis).as_euler("zyx")) * Affine(z=z_translation)
        point = self._reference_frame * point
        pitch, yaw, z_translation = self._pose_to_pyz(point)
        self.move_pyrz([pitch, yaw, roll, z_translation])

    def move_pyrz(self, pjrz: Union[list, tuple, np.array], degrees: bool = False):
        # prepare input
        pitch, yaw, roll, z_translation = pjrz
        if degrees:
            pitch = np.radians(pitch)
            yaw = np.radians(yaw)
            roll = np.radians(roll)
        roll = np.clip(roll, *self.roll_boundaries)
        z_translation = np.clip(z_translation, *self.z_translation_boundaries)
        current_pitch, current_yaw, current_roll, current_z_translation = self._pyrz
        current_eef_ppoint_distance = self.inital_eef_ppoint_distance - current_z_translation
        # calculate target pose
        new_eef_ppoint_distance = self.inital_eef_ppoint_distance - z_translation
        target_affine = self._reference_frame * Affine(b=yaw, c=pitch)
        target_affine *= Affine(z=z_translation)
        clipped, target_affine = self._clip_pose(target_affine)
        if clipped:
            pitch, yaw, _ = self._pose_to_pyz(target_affine)
        target_affine *= Affine(z=-self.inital_eef_ppoint_distance, a=roll)
        if self._visualize:
            self._publish_marker(target_affine, id=1)
        # move the robot
        # if relative z-translation is negative do it before pitch and yaw
        if new_eef_ppoint_distance > current_eef_ppoint_distance:
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

    def move_pyrz_relative(self, pjrz: Union[list, tuple, np.array], degrees: bool = False):
        pjrz = list(map(operator.add, self._pyrz, pjrz))
        self.move_pyrz(pjrz, degrees)
