#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from crazyflie_msgs.msg import AttitudeCommand
import numpy as np
import os
import yaml
import cflib
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie import Crazyflie
from typing import List
    
class MotorControlCommandNames:
    motor_control_group: str
    enable: str
    m1: str 
    m2: str 
    m3: str
    m4: str
    motors: List[str]

    def __init__(self) -> None:
        self.motor_control_group = "motorPowerSet"
        self.enable = "enable"
        self.m1 = "m1"
        self.m2 = "m2"
        self.m3 = "m3"
        self.m4 = "m4"
        self.motors = [self.m1, self.m2, self.m3, self.m4]


class MotorControl:
    _synced_cf: SyncCrazyflie
    _command_names: MotorControlCommandNames
    _debug_logs: bool

    def __init__(self, synced_cf: SyncCrazyflie) -> None:
        self._synced_cf = synced_cf
        self._command_names = MotorControlCommandNames()
        self._debug_logs = True
        self._setup_parameters()
       
    def _motor_enable_callback(self, name, value):
        if self._debug_logs:
            print(name, " is set to ", value)
    
    def _m_callback(self, name, value):
        if self._debug_logs:
            print(name, " is set to ", value) 


    def _setup_parameters(self):
        group = self._command_names.motor_control_group

        self._synced_cf.cf.param.add_update_callback(
                        group=group, 
                        name=self._command_names.enable,
                        cb=self._motor_enable_callback) 

        for motor in self._command_names.motors:
             self._synced_cf.cf.param.add_update_callback(
                        group=group, 
                        name=motor,
                        cb=self._m_callback) 


    def set_custom_callback(self, callback):
        group = self._command_names.motor_control_group

        self._synced_cf.cf.param.add_update_callback(
                        group=group, 
                        name=self._command_names.enable,
                        cb=callback) 

        for motor in self._command_names.motors:
             self._synced_cf.cf.param.add_update_callback(
                        group=group, 
                        name=motor,
                        cb=callback) 

    def enable_motor_control(self):
        full_name = self._command_names.motor_control_group + "." + self._command_names.enable
        self._synced_cf.cf.param.set_value(full_name, 1)

    def disable_motor_control(self):
        full_name = self._command_names.motor_control_group + "." + self._command_names.enable
        self._synced_cf.cf.param.set_value(full_name, 0)

    def set_motors(self, velocities: List[int]):
        assert len(velocities) == 4
        for i in range(4):
            full_name = self._command_names.motor_control_group + "." + self._command_names.motors[i]
            self._synced_cf.cf.param.set_value(full_name, velocities[i])


        


class CachedCfFactory:
    """
    Factory class that creates Crazyflie instances with TOC caching
    to reduce connection time.
    """

    def __init__(self, ro_cache=None, rw_cache=None):
        self.ro_cache = ro_cache
        self.rw_cache = rw_cache

    def construct(self, uri):
        cf = Crazyflie(ro_cache=self.ro_cache, rw_cache=self.rw_cache)
        return SyncCrazyflie(uri, cf=cf)

class ListenerNode(Node):
    _synced_cf: SyncCrazyflie
    """The crazyfly"""
    _factory: CachedCfFactory
    """For creating synced craziflie object."""
    _is_connected: bool
    """Whether crazyflie is connected."""
    _motor_control: MotorControl
    """Interface for low-level commands."""
    _is_ready: bool
    """Is drone ready to accept attitude commands."""
    
    def __init__(self):
        super().__init__('listener_node')

        # +++ flags        
        self._is_connected = False
        self._is_ready = False

        # +++ init drivers
        cflib.crtp.init_drivers()
        
        # +++ load cf config
        # crazyflies_yaml = os.path.join(
        #     os.curdir,
        #     'crazyflie_ros2_driver/crazyflies.yaml')
        # with open(crazyflies_yaml, 'r') as ymlfile:
        #     crazyflies = yaml.safe_load(ymlfile)
        # robot_data = crazyflies["robots"]
        # uris = [robot_data[crazyflie]["uri"] for crazyflie in robot_data if robot_data[crazyflie]["enabled"]]
        
        # +++ create synced cf
        factory = CachedCfFactory(rw_cache="./cache")
        self._synced_cf = factory.construct("radio://0/80/2M/E7E7E7E7E7")

        # +++ add callbacks
        self._synced_cf.cf.fully_connected.add_callback(self._fully_connected)
        self._synced_cf.cf.disconnected.add_callback(self._disconnected)
        self._synced_cf.cf.connection_failed.add_callback(self._connection_failed)

        # +++ open link to crazyfly
        try:
            self._synced_cf.open_link()
        except Exception as e:
            # Close node if one of the Crazyflies can not be found
            print("Error!: One or more Crazyflies can not be found. ")
            print("Check if you got the right URIs, if they are turned on" +
                                    " or if your script have proper access to a Crazyradio PA")

        # +++ subscribe to attitude commands
        self.subscription = self.create_subscription(
            AttitudeCommand,
           'crazyflie/pid/attitude_controller',
            self.set_attitude_callback,
            1)


    def _fully_connected(self, link_uri: str):
        """
        Called when connection to crazyflie is fully established
        """
        self._is_connected = True
        self.get_logger().debug("fully connected with " + link_uri)
        
        # disable motor level control
        self._motor_control = MotorControl(self._synced_cf)
        self._motor_control.set_custom_callback(self._motor_level_control_callback)
        self._motor_control.disable_motor_control()

    def _disconnected(self, link_uri):
        """
        Called when disconnected
        """
        self._is_connected = False
        self.get_logger().info("disconnected with " + link_uri)

    def _connection_failed(self, link_uri, msg):
        """
        Called when the connection failed
        """
        self._is_connected = False
        self.get_logger().error("connection failed with " + link_uri)
        self.get_logger().error(msg)

    def _motor_level_control_callback(self, name: str, value: any):
        """
        Called when motor level control is debugged
        """
        self._is_ready = True
        self.get_logger().debug("motor level control callback: " + name + str(value))
   

    def set_attitude_callback(self, attitude_msg: AttitudeCommand):
        """
        Set attitude
        """
        thrust_in = attitude_msg.thurst if attitude_msg.thurst <= 10 else 10
        thrust_in = thrust_in if thrust_in > 2.5 else 0
        thrust_in = 10001 + thrust_in * (3000) if thrust_in != 0 else 0

        if self._is_connected and self._is_ready:
            self.get_logger().debug('I heard: ' + str(attitude_msg.thurst))
            self._synced_cf.cf.commander.send_setpoint(
                            float(attitude_msg.pitch), 
                            float(attitude_msg.roll), 
                            float(attitude_msg.yaw), 
                            int(thrust_in))
            


def main(args=None):
    rclpy.init(args=args)
    node = ListenerNode()

    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
