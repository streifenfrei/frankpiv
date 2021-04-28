import operator
import sys
from abc import ABC
from typing import Union

import moveit_commander
import numpy as np
import rospy
from cfrankr import Affine
from scipy.spatial.transform.rotation import Rotation

from frankpiv.backend.conversions import pose_msg_to_affine, affine_to_pose_msg
from frankpiv.backend.general import GeneralBackend, UnattainablePoseException


class Backend(GeneralBackend, ABC):
    def __init__(self, config):
        super().__init__(config)
        # configuration
        self.moveit_config = config["moveit"]
        self.inital_eef_ppoint_distance = config["eef_ppoint_distance"]
        self.tool_length = config["tool_length"]
        self.max_angle = np.radians(config["max_angle"])
        self.roll_boundaries = np.radians(config["roll_boundaries"])
        self.z_translation_boundaries = config["z_translation_boundaries"]
        self.clip_to_boundaries = config["clip_to_boundaries"]
        self.move_directly = config["move_directly"]
        # robotic stuff
        self._robot = None
        self._reference_frame = None
        self._pyrz = None

    def start(self):
        super().start()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node(self._node_name, anonymous=True)
        if self._robot is None:
            self._robot = moveit_commander.MoveGroupCommander(self.moveit_config["robot_name"])
        self._reference_frame = pose_msg_to_affine(self._robot.get_current_pose()) * \
                                Affine(z=self.inital_eef_ppoint_distance)
        self._pyrz = [0, 0, 0, 0]
        if self._visualize:
            self._publish_marker(self._reference_frame, id=0)

    def stop(self):
        super().stop()
        self._robot = None
        rospy.signal_shutdown("done")
        moveit_commander.roscpp_shutdown()

    def _clip_pose(self, pose: Affine):
        pose_local = pose
        point = pose_local.to_array()[:3]
        distance = np.linalg.norm(point)
        z_translation = distance if point[2] >= 0 else -distance
        z_axis = [0, 0, 1] if point[2] >= 0 else [0, 0, -1]
        if point[0] == point[1] == point[2] == 0.0:  # point lies in the origin -> add offset to calculate angle
            point_offset = (pose_local * Affine(z=1)).to_array()[:3]
            angle = np.inner(point_offset, z_axis)
        else:
            angle = np.inner(point, z_axis) / z_translation
        angle = -np.arccos(angle)
        # clip pose if angle or z-translation are outside the specified boundaries
        if angle < -self.max_angle or angle > self.max_angle or \
                z_translation < self.z_translation_boundaries[0] or z_translation > self.z_translation_boundaries[1]:
            # calculate clipped point by rotating along the rotation axis and translate along the z axis
            angle = np.clip(angle, -self.max_angle, self.max_angle)
            z_translation = np.clip(z_translation, *self.z_translation_boundaries)
            rotation_axis = np.cross(point, z_axis)
            rotation_axis = rotation_axis / np.linalg.norm(rotation_axis) * angle
            clipped_pose = Affine(0, 0, 0, *Rotation.from_rotvec(rotation_axis).as_euler("zyx")) * \
                           Affine(z=z_translation)
            return True, clipped_pose
        return False, pose

    @staticmethod
    def _pose_to_pyz(pose: Affine):
        pose = pose.to_array()
        distance = np.linalg.norm(pose[:3])
        z_translation = distance if pose[2] >= 0 else -distance
        return pose[5], pose[4], z_translation

    def move_to_point(self, point: Union[list, tuple, np.array], roll: float,
                      frame: Union[list, tuple, np.array] = None):
        if roll < self.roll_boundaries[0] or roll > self.roll_boundaries[1]:
            raise UnattainablePoseException("Roll value is outside of specified boundaries")
        roll = np.clip(roll, *self.roll_boundaries)
        point = Affine(x=point[0], y=point[1], z=point[2])
        if frame is not None:
            # convert target point to the reference frame
            point = Affine(frame) * point
            point = (self._reference_frame.inverse() * point)
        point = point.to_array()[:3]
        if point[0] == point[1] == point[2] == 0.0:  # target point is the origin
            pose = Affine()
        else:
            if point[2] < 0:
                raise UnattainablePoseException("Point must have positive z value in the pivot point frame")
            distance = np.linalg.norm(point)
            angle = -np.arccos(np.inner(point, [0, 0, 1]) / distance)
            if angle == 0.0:  # target point lies on the z axis
                pose = Affine()
            else:
                # we rotate and translate to the target point to get the target rotation
                rotation_axis = np.cross(point, [0, 0, 1])
                rotation_axis = rotation_axis / np.linalg.norm(rotation_axis) * angle
                pose = Affine(0, 0, 0, *Rotation.from_rotvec(rotation_axis).as_euler("zyx"))
            z_translation = distance - self.tool_length + self.inital_eef_ppoint_distance
            pose *= Affine(z=z_translation)
        if self.move_directly:
            clipped, pose = self._clip_pose(pose)
            if clipped and not self.clip_to_boundaries:
                raise UnattainablePoseException("Target point lies outside of specified boundaries")
            # translate to the end effector pose and convert to global frame
            target_pose = self._reference_frame * (pose * Affine(z=-self.inital_eef_ppoint_distance, a=roll))
            if self._visualize:
                self._publish_marker(target_pose, id=1)
                self._publish_marker(target_pose * Affine(z=self.tool_length), type=self.MarkerType.Point, id=2)
            self._robot.go(affine_to_pose_msg(target_pose))
            if self._visualize:
                self._delete_marker(id=1)
                self._delete_marker(id=2)
            pitch, yaw, z_translation = self._pose_to_pyz(pose)
            self._pyrz = [pitch, yaw, roll, z_translation]
        else:
            pitch, yaw, z_translation = self._pose_to_pyz(pose)
            self.move_pyrz([pitch, yaw, roll, z_translation])

    def move_pyrz(self, pjrz: Union[list, tuple, np.array], degrees: bool = False):
        # prepare input
        pitch, yaw, roll, z_translation = pjrz
        if degrees:
            pitch = np.radians(pitch)
            yaw = np.radians(yaw)
            roll = np.radians(roll)
        if not self.clip_to_boundaries:
            if roll < self.roll_boundaries[0] or roll > self.roll_boundaries[1]:
                raise UnattainablePoseException("Roll value is outside of specified boundaries")
            if z_translation < self.z_translation_boundaries[0] or z_translation > self.z_translation_boundaries[1]:
                raise UnattainablePoseException("Z-translation value is outside of specified boundaries")
        roll = np.clip(roll, *self.roll_boundaries)
        z_translation = np.clip(z_translation, *self.z_translation_boundaries)
        # calculate target pose
        new_eef_ppoint_distance = self.inital_eef_ppoint_distance - z_translation
        target_affine = Affine(b=yaw, c=pitch) * Affine(z=z_translation)
        clipped, target_affine = self._clip_pose(target_affine)
        if clipped:
            if not self.clip_to_boundaries:
                raise UnattainablePoseException("Target point lies outside of specified boundaries")
            pitch, yaw, _ = self._pose_to_pyz(target_affine)
        target_affine = self._reference_frame * (target_affine * Affine(z=-self.inital_eef_ppoint_distance, a=roll))
        # move the robot
        if self._visualize:
            self._publish_marker(target_affine, id=1)
            self._publish_marker(target_affine * Affine(z=self.tool_length), type=self.MarkerType.Point, id=2)
        if not self.move_directly:
            current_pitch, current_yaw, current_roll, current_z_translation = self._pyrz
            current_eef_ppoint_distance = self.inital_eef_ppoint_distance - current_z_translation
            # if relative z-translation is negative do it before pitch and yaw
            if new_eef_ppoint_distance > current_eef_ppoint_distance:
                intermediate_affine = self._reference_frame * Affine(b=current_yaw, c=current_pitch) * \
                                      Affine(z=-new_eef_ppoint_distance, a=roll)
                self._robot.go(affine_to_pose_msg(intermediate_affine))
            else:
                intermediate_affine = self._reference_frame * Affine(b=yaw, c=pitch) * \
                                      Affine(z=-current_eef_ppoint_distance, a=current_roll)
                self._robot.go(affine_to_pose_msg(intermediate_affine))
            self._robot.go(affine_to_pose_msg(target_affine))
        self._pyrz = [pitch, yaw, roll, z_translation]
        if self._visualize:
            self._delete_marker(id=1)
            self._delete_marker(id=2)

    def move_pyrz_relative(self, pjrz: Union[list, tuple, np.array], degrees: bool = False):
        pjrz = list(map(operator.add, self._pyrz, pjrz))
        self.move_pyrz(pjrz, degrees)
