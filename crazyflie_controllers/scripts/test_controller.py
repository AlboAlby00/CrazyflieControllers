#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from crazyflie_msgs.msg import MotorVel
from time import sleep



class TestController(Node):
    
    def __init__(self):
        super().__init__('test_controller')
        self._cmd_motor_vel_pub = self.create_publisher(MotorVel, 'crazyflie/cmd_motor_vel', 10)
        self._first_time = True

        sleep(3)

        timer_period = 2  
        self._timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        if self._first_time:
            propeller_vel = 500.0
            self.get_logger().info('Publishing cmd_motor_vel for the first time')
            self._first_time = False
        else:
            propeller_vel = 53.0
        
        msg = MotorVel()
        msg.m1 =   propeller_vel
        msg.m2 =   propeller_vel
        msg.m3 =   propeller_vel
        msg.m4 =   propeller_vel
        self._cmd_motor_vel_pub.publish(msg)
        


def main(args=None):
    rclpy.init(args=args)
    controller = TestController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
