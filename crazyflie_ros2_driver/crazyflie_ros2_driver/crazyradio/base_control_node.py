#!/usr/bin/env python3
from rclpy.node import Node
import cflib
import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from .motor_control import MotorControl
from .cached_cf_factory import CachedCfFactory


class BaseControlNode(Node):
    """
    Requires child class to call _add_standard_callbacks 
    """

    _synced_cf: SyncCrazyflie
    """The crazyfly"""
    _factory: CachedCfFactory
    """For creating synced craziflie object."""
    _is_connected: bool
    """Whether crazyflie is connected."""
    _motor_control: MotorControl
    """Interface for low-level commands."""
    
    
    def __init__(self):
        super().__init__('listener_node')

        # +++ flags        
        self._is_connected = False

        # +++ init drivers
        cflib.crtp.init_drivers()
        
        # +++ create synced cf
        factory = CachedCfFactory(rw_cache="./cache")
        self._synced_cf = factory.construct("radio://0/80/2M/E7E7E7E7E7")
    
    def _add_standard_callbacks(self):
        self._synced_cf.cf.fully_connected.add_callback(self._fully_connected)
        self._synced_cf.cf.disconnected.add_callback(self._disconnected)
        self._synced_cf.cf.connection_failed.add_callback(self._connection_failed) 

    def _open_link(self):
        # +++ open link to crazyfly
        try:
            self._synced_cf.open_link()
        except Exception as e:
            # Close node if one of the Crazyflies can not be found
            print("Error!: One or more Crazyflies can not be found. ")
            print("Check if you got the right URIs, if they are turned on" +
                                    " or if your script have proper access to a Crazyradio PA")


    def _fully_connected(self, link_uri: str):
        """
        Called when connection to crazyflie is fully established
        """
        self._is_connected = True
        self.get_logger().debug("fully connected with " + link_uri)
        
        # initialize motor level control
        self._motor_control = MotorControl(self._synced_cf)
        

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

    