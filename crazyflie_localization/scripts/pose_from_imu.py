#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Point
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.qos import QoSProfile

class PoseFromImu(Node):
    def __init__(self):
        super().__init__('pose_from_imu')
        self._acceleration = Point()
        self._velocity = Twist()
        self._position = Point()
        self._old_time = self.get_clock().now()

        self._sub_imu = self.create_subscription(
            Imu,
            '/crazyflie/imu',
            self.imu_callback,
            10)

        self._pub_imu_pose = self.create_publisher(
            Point,
            '/crazyflie/imu_pose',
            QoSProfile(depth=10))

        self.timer = self.create_timer(0.1, self.publish_position)

        self.get_logger().info("pose_from_imu node is running")

    def imu_callback(self, msg):
        # Assuming IMU data contains linear acceleration in m/s^2
        self._acceleration.x = msg.linear_acceleration.x
        self._acceleration.y = msg.linear_acceleration.y
        self._acceleration.z = msg.linear_acceleration.z
        self.get_logger().debug("imu data received")


    def publish_position(self):
        dt : Duration = self._old_time - self.get_clock().now() 
        self.get_logger().info(str(dt.nanoseconds * 1e-9))
        # Integrate acceleration to obtain velocity
        self._velocity.linear.x += self._acceleration.x * dt.nanoseconds * 1e-9
        self._velocity.linear.y += self._acceleration.y * dt.nanoseconds * 1e-9
        self._velocity.linear.z += self._acceleration.z * dt.nanoseconds * 1e-9

        # Integrate velocity to obtain position
        self._position.x += self._velocity.linear.x * dt.nanoseconds * 1e-9
        self._position.y += self._velocity.linear.y * dt.nanoseconds * 1e-9
        self._position.z += self._velocity.linear.z * dt.nanoseconds * 1e-9

        self._old_time = self.get_clock().now() 

        # Publish the position
        self._pub_imu_pose.publish(self._position)

def main(args=None):
    rclpy.init(args=args)
    imu_integration_node = PoseFromImu()
    rclpy.spin(imu_integration_node)
    imu_integration_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()