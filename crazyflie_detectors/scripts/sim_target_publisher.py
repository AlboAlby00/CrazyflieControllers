#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from crazyflie_msgs.msg import TargetTransformation
from geometry_msgs.msg import PointStamped
import random


class TargetPublisher(Node):
    def __init__(self):
        super().__init__('crazyflie_detectors')
        self.publisher = self.create_publisher(TargetTransformation, 'crazyflie/target_transformation', 10)
        self.gps_sub = self.create_subscription(PointStamped, 'crazyflie/gps', self.gps_callback, 10)

        self.target = [random.uniform(-5, 5), random.uniform(-5, 5), 0.0]
        self.translation = [0.0, 0.0, 0.0]
        self.rotation = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
    
    def gps_callback(self, msg):
        self.gps_x = msg.point.x
        self.gps_y = msg.point.y
        self.gps_z = msg.point.z

        # publish target vector
        self.translation = [
            self.target[0] - self.gps_x,
            self.target[1] - self.gps_y,
            self.target[2] - self.gps_z
        ]
        transform = TargetTransformation()
        transform.t = self.translation + self.rotation

        self.publisher.publish(transform)
        self.get_logger().info('Publishing: "%s"' % transform)


def main(args=None):
    rclpy.init(args=args)
    target_publisher = TargetPublisher()
    rclpy.spin(target_publisher)

if __name__ == '__main__':
    main()
