#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from crazyflie_msgs.msg import TargetTransformationVector
from geometry_msgs.msg import PointStamped
import random



class TargetPublisher(Node):
    def __init__(self):
        super().__init__('crazyflie_detectors')
        self.publisher = self.create_publisher(TargetTransformationVector, 'crazyflie/target_transformation_vector', 10)
        timer_period = 1.0 # seconds
        self.timer = self.create_timer(timer_period, self.publish_target)

        self.x_target = random.uniform(-5, 5)
        self.y_target = random.uniform(-5, 5)
        self.z_target = 0.0

        # create gps subscriber
        self.gps_sub = self.create_subscription(PointStamped, 'crazyflie/gps', self.gps_callback, 10)
    
    def publish_target(self):
        msg = TargetTransformationVector()
        msg.x = self.x_target
        msg.y = self.y_target
        msg.z = self.z_target
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)
    
    def gps_callback(self, msg):
        self.gps_x = msg.point.x
        self.gps_y = msg.point.y
        self.gps_z = msg.point.z

        # publish target vector
        vec_to_target = TargetTransformationVector()
        vec_to_target.x = self.x_target - self.gps_x
        vec_to_target.y = self.y_target - self.gps_y
        vec_to_target.z = self.z_target - self.gps_z

        self.publisher.publish(vec_to_target)
        # log vec_to_target
        self.get_logger().info('Publishing: "%s"' % vec_to_target)


def main(args=None):
    rclpy.init(args=args)
    target_publisher = TargetPublisher()
    rclpy.spin(target_publisher)

if __name__ == '__main__':
    main()
