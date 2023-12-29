#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.qos import qos_profile_sensor_data


class EkfWrapper(Node):

    def __init__(self):

        super().__init__('ekf_wrapper')
        self.get_logger().info("ekf wrapper running")
        self.sub_camera_position = self.create_subscription(PointStamped, 'crazyflie/camera_position',
                                                            self.publish_visual_odometry, qos_profile=qos_profile_sensor_data)
        self.sub_filtered_odometry = self.create_subscription(Odometry, 'crazyflie/filtered_odometry',
                                                              self.publish_filtered_position, qos_profile=qos_profile_sensor_data)
        self.sub_imu = self.create_subscription(Imu, 'crazyflie/imu', self.publish_imu_with_covariance, qos_profile=qos_profile_sensor_data)
        self.pub_camera_pos = self.create_publisher(Odometry, 'crazyflie/visual_odometry', 10)

        self.pub_filtered_position = self.create_publisher(PointStamped, 'crazyflie/filtered_position', 10)
        self.pub_imu_with_covariance = self.create_publisher(Imu, 'crazyflie/imu_with_covariance', 10)

    def publish_visual_odometry(self, msg: PointStamped):
        new_msg = Odometry()
        new_msg.header.frame_id = "base_link"
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.pose.pose.position.x = msg.point.x
        new_msg.pose.pose.position.y = msg.point.y
        new_msg.pose.pose.position.z = msg.point.z
        new_msg.pose.covariance = [1e-4,   0.0,      0.0,    0.0,    0.0,    0.0,
                                   0.0,    1e-4,     0.0,    0.0,    0.0,    0.0,
                                   0.0,    0.0,      1e-4,   0.0,    0.0,    0.0,
                                   0.0,    0.0,      0.0,    1e-4,    0.0,    0.0,
                                   0.0,    0.0,      0.0,    0.0,    1e-4,    0.0,
                                   0.0,    0.0,      0.0,    0.0,    0.0,    1e-4
                                   ]
        self.pub_camera_pos.publish(new_msg)

    def publish_imu_with_covariance(self, msg: Imu):
        new_msg = Imu()
        new_msg.header.frame_id = "base_link"
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.angular_velocity = msg.angular_velocity
        new_msg.linear_acceleration = msg.linear_acceleration
        new_msg.orientation = msg.orientation
        # Set covariance values (example values, replace with your own)
        new_msg.orientation_covariance = [0.01, 0.0, 0.0,
                                          0.0, 0.01, 0.0,
                                          0.0, 0.0, 0.01]
        new_msg.angular_velocity_covariance = [0.01, 0.0, 0.0,
                                               0.0, 0.01, 0.0,
                                               0.0, 0.0, 0.01]
        new_msg.linear_acceleration_covariance = [0.01, 0.0, 0.0,
                                                  0.0, 0.01, 0.0,
                                                  0.0, 0.0, 0.01]
        self.pub_imu_with_covariance.publish(new_msg)

    def publish_filtered_position(self, msg: PointStamped):
        new_msg = Odometry()
        new_msg.pose.pose.position.x = msg.pose.pose.position.x
        new_msg.pose.pose.position.y = msg.pose.pose.position.y
        new_msg.pose.pose.position.z = msg.pose.pose.position.z
        self.pub_filtered_position.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = EkfWrapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
