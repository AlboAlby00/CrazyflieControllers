#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PointStamped
import numpy as np
from rclpy.qos import qos_profile_sensor_data


class CameraAligner(Node):

    def __init__(self):

        super().__init__('camera_aligner')
        self.get_logger().info("camera aligner running")
        self.sub_camera_pos = self.create_subscription(PointStamped, 'crazyflie/camera_position_disaligned',
                                                       self.align_callback, qos_profile=qos_profile_sensor_data)
        self.pub_camera_pos = self.create_publisher(PointStamped, 'crazyflie/camera_position', 10)

        self.declare_parameter('sim_camera', True)
        self.sim_camera = self.get_parameter('sim_camera').get_parameter_value().bool_value

    def align_callback(self, msg: PointStamped):
        new_msg = PointStamped()
        new_msg.header = msg.header
        """
        numpy_point = np.array([msg.point.x, msg.point.y, msg.point.z])
        rotated_point = self.rotation @ numpy_point
        new_msg.point.x = rotated_point[0]
        new_msg.point.y = rotated_point[1]
        new_msg.point.z = rotated_point[2]
        """
        if self.sim_camera:
            new_msg.point.x = - msg.point.z
            new_msg.point.y = msg.point.x
            new_msg.point.z = msg.point.y
            self.pub_camera_pos.publish(new_msg)
        else:
            # TODO
            new_msg.point.x = msg.point.x
            new_msg.point.y = msg.point.y
            new_msg.point.z = msg.point.z
            self.pub_camera_pos.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraAligner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
