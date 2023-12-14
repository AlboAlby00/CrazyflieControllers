#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from crazyflie_msgs.msg import AttitudeCommand

class CmdVelToAttitudeConverter(Node):

    def __init__(self) -> None:
        super().__init__('cmd_vel_to_attitude')
        self._cmd_vel_sub =  self._sub_new_position = self.create_subscription(
                Twist ,"cmd_vel", self._new_cmd_vel_callback,10)
        self._attitude_command_pub =  self.create_publisher(
                AttitudeCommand, "/crazyflie/pid/attitude_controller", 10 )
        self._pitch = 0
        self._roll = 0
        self._yaw = 0
        self._thurst = 1
        self.get_logger().info("cmd_vel_to_attitude node is running!")

    def _new_cmd_vel_callback(self, cmd_vel : Twist):

        self._pitch += self.deg_to_radiants(cmd_vel.linear.x)
        self._roll += self.deg_to_radiants(cmd_vel.linear.y)
        self._yaw += self.deg_to_radiants(cmd_vel.linear.z) * 10
        self._thurst += cmd_vel.angular.z

        self.get_logger().info(f"new attitude command! \
            [p: {self._pitch}, r: {self._roll}, y: {self._yaw}, t: {self._thurst}]")

        attitude_command = AttitudeCommand()
        attitude_command.thurst = self._thurst
        attitude_command.pitch = self._pitch
        attitude_command.roll = self._roll 
        attitude_command.yaw = self._yaw 

        self._attitude_command_pub.publish(attitude_command)

        return True

    def deg_to_radiants(self, angle):
        return angle * 2 * 3.14 / 360


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelToAttitudeConverter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
