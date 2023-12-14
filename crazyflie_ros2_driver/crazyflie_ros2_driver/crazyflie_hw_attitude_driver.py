#!/usr/bin/env python3
import rclpy
from crazyflie_msgs.msg import AttitudeCommand
from .crazyradio.base_control_node import BaseControlNode
import math
from example_interfaces.srv import AddTwoInts, SetBool
GRAVITY_COMPENSATION = 40000

def convert_thrust(in_: float):
    """
    - converts scalar from 2.5 to 10 to range of 10001 to 60000
    - returns 0 if in_ is 0
    - returns int
    """
    thrust_in = in_ if in_ <= 10 else 10
    thrust_in = thrust_in - 2.5 if thrust_in > 2.5 else 0
    thrust_in = 10001 + thrust_in * ((42000 - 10001) / (7.5)) if thrust_in != 0 else 0
    return int(thrust_in)

def compute_thrust(gravity_compenstation: int, in_thrust: float):
    return int(gravity_compenstation + in_thrust)

def get_factor(in_: float):
    thrust_in = 0 if in_ <= 2.5 else 1 
    return thrust_in

def radian_to_degree(in_: float):
    return float((in_ / (math.pi)) * 360)

class AttitudeControlNode(BaseControlNode):
    gravity_compensation: int
    _activate_motors: bool
    _is_ready: bool
    """Is drone ready to accept attitude commands."""

    def __init__(self):
        super().__init__()
        self._activate_motors = False
        self.gravity_compensation = GRAVITY_COMPENSATION
        self._is_ready = False

        # +++ add callbacks
        self._synced_cf.cf.fully_connected.add_callback(self._fully_connected_motor_level)
        self._synced_cf.cf.disconnected.add_callback(self._disconnected)
        self._synced_cf.cf.connection_failed.add_callback(self._connection_failed)

        # +++ open link
        self._open_link()

        # +++ initialise subscription
        self.__initialize_subscription()

        self.srv = self.create_service(AddTwoInts, 'set_gravity_compensation', self.set_gravity_compensation)
        self.srv_activate = self.create_service(SetBool, 'activate_motors', self.activate_motors)

    def activate_motors(self, request, response):
        # self.get_logger().info("received activate motors command" + str(request.data) + "\n")
        self.get_logger().info("activate motors\n")
        self._activate_motors = bool(request.data)
        response.success = True
        response.message = "Activate motors command processed successfully."
        return response

    def set_gravity_compensation(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info('Incoming request\na: %d b: %d' % (request.a, request.b))
        self.gravity_compensation = response.sum
        return response

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
        thrust_in = compute_thrust(int(self.gravity_compensation), attitude_msg.thurst) if self._activate_motors else 0 # + convert_thrust(attitude_msg.thurst)
        self.get_logger().info("Received attitude command, thrust: " + str(thrust_in))

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
