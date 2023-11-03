#!/usr/bin/env python3

"""
"The 'buttons' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from crazyflie_msgs.msg import AttitudeCommand
from sensor_msgs.msg import Joy

import numpy as np

class CmdVelToAttitudeConverter(Node):

    def __init__(self) -> None:
        super().__init__('joystick_to_attitude')
        self._joy_sub =  self._sub_new_position = self.create_subscription(
                Joy ,"joy", self._new_cmd_vel_callback,10)
        self._attitude_command_pub =  self.create_publisher(
                AttitudeCommand, "/crazyflie/pid/attitude_controller", 10 )
        self.get_logger().info("joystick_to_attitude node is running!")
        
    def _new_cmd_vel_callback(self, cmd_joy : Joy):

        attitude_command = AttitudeCommand()
        attitude_command.thurst = np.interp(cmd_joy.axes[1], (-1, 1), (- 5, 10))
        attitude_command.pitch = - np.interp(cmd_joy.axes[3], (-1, 1), (- np.pi / 36, np.pi / 36))
        attitude_command.roll = np.interp(cmd_joy.axes[4], (-1, 1), (- np.pi / 36, np.pi / 36)) 
        button_right = cmd_joy.buttons[len(cmd_joy.buttons) - 1]
        button_left = cmd_joy.buttons[len(cmd_joy.buttons) - 2]

        # compute yaw
        yaw_ = 0.0
        if button_left or button_right:
            yaw_ = 1.0 if button_left else -1.0            
        attitude_command.yaw = yaw_ # np.interp(cmd_joy.axes[0], (-1, 1), (0, 2 * np.pi))
        
        # publish command
        self._attitude_command_pub.publish(attitude_command)

        return True



def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAttitudeConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()