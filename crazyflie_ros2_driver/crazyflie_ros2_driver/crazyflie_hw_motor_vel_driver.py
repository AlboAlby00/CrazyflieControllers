#!/usr/bin/env python3
import rclpy
from crazyflie_msgs.msg import MotorVel
from .crazyradio.base_control_node import BaseControlNode

def map_velocities(in_: float):
    """
    f: [0,1] /rightarrow [10001, 60000] /union /mathbb{N}
    """
    in_ = in_ if in_ > 0 else 0
    in_ = in_ if in_ < 1 else 1
    # 60000 - 10001 = 49999
    in_ = 10001 + in_ * 49999 if in_ != 0 else 0
    return int(in_)

class MotorLevelControlNode(BaseControlNode):
    """Is drone ready to accept motor velocity commands."""
    _is_ready: bool 
    def __init__(self):
        super().__init__()
        self.get_logger().info("starting motor level control")
        self._is_ready = False

        # +++ add callbacks
        self._synced_cf.cf.fully_connected.add_callback(self._fully_connected_callback_motor_level)
        self._synced_cf.cf.disconnected.add_callback(self._disconnected)
        self._synced_cf.cf.connection_failed.add_callback(self._connection_failed) 

        # +++ open link
        self._open_link()

        # +++ initialise subscription
        self.__initialize_subscription()

    def _fully_connected_callback_motor_level(self, link_uri: str):
        super()._fully_connected(link_uri)
        
        self._motor_control.set_custom_callback_enable_motor_level_control(self._enable_motor_level_ctrl_callback)
        self._motor_control.enable_motor_control()


    def _enable_motor_level_ctrl_callback(self, name, value):
        self.get_logger().debug("callback: " + name + "; value: " + str(value))
        self._is_ready = True

    def _set_motor_vel_callback(self, name, value):
        self.get_logger().debug("callback: " + name + "; value: " + str(value))

    def __initialize_subscription(self):
        # +++ subscribe to attitude commands
        self.subscription = self.create_subscription(
            MotorVel,
           'crazyflie/cmd_motor_vel',
            self.set_motor_vel_callback,
            1)

    def set_motor_vel_callback(self, motor_cmd: MotorVel):
        """
        Set motor velocities
        """
        # self.get_logger().info("received m1: " + str(motor_cmd.m1))
        # self.get_logger().info("received m2: " + str(motor_cmd.m2))
        # self.get_logger().info("received m3: " + str(motor_cmd.m3))
        # self.get_logger().info("received m4: " + str(motor_cmd.m4))
        if self._is_ready:
            self._motor_control.set_motors([
                map_velocities(motor_cmd.m1), 
                map_velocities(motor_cmd.m2), 
                map_velocities(motor_cmd.m3), 
                map_velocities(motor_cmd.m4)])
    

    
def main(args=None):
    rclpy.init(args=args)
    node = MotorLevelControlNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
