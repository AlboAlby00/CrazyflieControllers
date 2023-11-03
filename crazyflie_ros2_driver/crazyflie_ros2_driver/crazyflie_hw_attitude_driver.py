#!/usr/bin/env python3
import rclpy
from crazyflie_msgs.msg import AttitudeCommand
from .crazyradio.base_control_node import BaseControlNode
from .crazyradio.tools import convert_thrust, radian_to_degree


class AttitudeControlNode(BaseControlNode):
    _is_ready: bool
    """Is drone ready to accept attitude commands."""

    def __init__(self):
        super().__init__()
        self._is_ready = False

        # +++ add callbacks
        self._synced_cf.cf.fully_connected.add_callback(self._fully_connected_motor_level)
        self._synced_cf.cf.disconnected.add_callback(self._disconnected)
        self._synced_cf.cf.connection_failed.add_callback(self._connection_failed)

        # +++ open link
        self._open_link()

        # +++ initialise subscription
        self.__initialize_subscription()

    def _fully_connected_motor_level(self, link_uri: str):
        super()._fully_connected(link_uri)
        self._motor_control.set_custom_callback(self._motor_level_control_callback)
        self._motor_control.disable_motor_control()

    def __initialize_subscription(self):
        # +++ subscribe to attitude commands
        self.subscription = self.create_subscription(
            AttitudeCommand,
           'crazyflie/pid/attitude_controller',
            self.set_attitude_callback,
            1)

    def set_attitude_callback(self, attitude_msg: AttitudeCommand):
        """
        Set attitude
        """
        thrust_in = convert_thrust(attitude_msg.thurst)

        if self._is_connected and self._is_ready:
            self.get_logger().debug('Received thrust: ' + str(attitude_msg.thurst))
            self.get_logger().debug("Received pitch: " + str(attitude_msg.pitch))
            self.get_logger().debug("Received roll: " + str(attitude_msg.roll))
            self.get_logger().debug("Received yaw: " + str(attitude_msg.yaw))
            self._synced_cf.cf.commander.send_setpoint(
                            radian_to_degree(attitude_msg.pitch), 
                            radian_to_degree(attitude_msg.roll), 
                            radian_to_degree(attitude_msg.yaw), 
                            thrust_in)
    
    def _motor_level_control_callback(self, name: str, value: any):
        """
        Called when motor level control is debugged
        """
        self._is_ready = True
        self.get_logger().debug("motor level control callback: " + name + str(value))

    
def main(args=None):
    rclpy.init(args=args)
    node = AttitudeControlNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
