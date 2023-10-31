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

        self.x_target = random.uniform(-5, 5)
        self.y_target = random.uniform(-5, 5)
        self.z_target = 0
    
    def publish_target(self):
        msg = TargetTransformationVector()
        # random float64 from range -5 to 5
        msg.x = self.x_target
        msg.y = self.y_target
        msg.z = self.z_target
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    target_publisher = TargetPublisher()
    rclpy.spin(target_publisher)

if __name__ == '__main__':
    main()
