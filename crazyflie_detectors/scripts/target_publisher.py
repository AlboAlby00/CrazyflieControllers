#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from crazyflie_msgs.msg import TargetTransformationVector
import random



class TargetPublisher(Node):
    def __init__(self):
        super().__init__('crazyflie_detectors')
        self.publisher = self.create_publisher(TargetTransformationVector, 'crazyflie/target_transformation_vector', 10)
        timer_period = 1.0 # seconds
        self.timer = self.create_timer(timer_period, self.publish_target)
    
    def publish_target(self):
        msg = TargetTransformationVector()
        # random float64
        msg.x = random.random()
        msg.y = random.random()
        msg.z = random.random()
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    target_publisher = TargetPublisher()
    rclpy.spin(target_publisher)

if __name__ == '__main__':
    main()
