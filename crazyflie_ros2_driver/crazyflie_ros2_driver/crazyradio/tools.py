import math
from geometry_msgs.msg import Quaternion

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    """
    c1 = math.cos(yaw   / 2)
    c2 = math.cos(pitch / 2)
    c3 = math.cos(roll  / 2)
    s1 = math.sin(yaw   / 2)
    s2 = math.sin(pitch / 2)
    s3 = math.sin(roll  / 2)

    quaternion = Quaternion()
    quaternion.w = c1 * c2 * c3 - s1 * s2 * s3
    quaternion.x = s1 * s2 * c3 + c1 * c2 * s3
    quaternion.y = s1 * c2 * c3 + c1 * s2 * s3
    quaternion.z = c1 * s2 * c3 + s1 * c2 * s3
    
    return quaternion
