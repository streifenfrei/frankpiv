from cfrankr import Affine
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion
from scipy.spatial.transform.rotation import Rotation


def pose_msg_to_affine(msg):
    if isinstance(msg, PoseStamped):
        msg = msg.pose
    position = [msg.position.x, msg.position.y, msg.position.z]
    rotation = Rotation.from_quat([msg.orientation.x, msg.orientation.y,
                                   msg.orientation.z, msg.orientation.w]).as_euler("ZYX")
    return Affine(*position, *rotation)


def affine_to_pose_msg(affine):
    array = affine.to_array()
    msg = Pose(position=Point(*array[:3]), orientation=Quaternion(*Rotation.from_euler("ZYX", array[3:]).as_quat()))
    return msg
