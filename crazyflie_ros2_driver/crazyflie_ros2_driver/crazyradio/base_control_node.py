#!/usr/bin/env python3

"""
[crazyflie_hw_attitude_driver-1] KeyError: 'Variable imu_sensors.imuroll not in TOC'

Python 3.8.10 (default, May 26 2023, 14:05:08) 
[GCC 9.4.0] on linux
Type "help", "copyright", "credits" or "license" for more information.
>>> from sensor_msgs.msg import Imu
>>> imu = Imu()
>>> imu.get_fields_and_field_types()
{'header': 'std_msgs/Header', 'orientation': 'geometry_msgs/Quaternion', 'orientation_covariance': 'double[9]', 'angular_velocity': 'geometry_msgs/Vector3', 'angular_velocity_covariance': 'double[9]', 'linear_acceleration': 'geometry_msgs/Vector3', 'linear_acceleration_covariance': 'double[9]'}
"""

from rclpy.node import Node
import cflib
import cflib.crtp
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from .motor_control import MotorControl
from .cached_cf_factory import CachedCfFactory
import time 
from cflib.crazyflie.log import LogConfig
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from crazyflie_msgs.msg import EulerAngle
from typing import List
from cflib.crazyflie.log import LogVariable
import math 

def join_group_and_name(group, name):
    return group + "." + name

def log_stab_callback(timestamp, data, logconf):
    print('[%d][%s]: %s' % (timestamp, logconf.name, data))

def simple_log_async(scf, logconf):
    cf = scf.cf
    cf.log.add_config(logconf)
    logconf.data_received_cb.add_callback(log_stab_callback)
    logconf.start()
    time.sleep(5)
    logconf.stop()

def euler_to_quaternion(roll: float, pitch: float, yaw: float) -> Quaternion:
    """
    https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    """
    c1 = math.cos(yaw   / 2)
    c2 = math.cos(pitch / 2)
    c3 = math.cos(roll  / 2)
    s1 = math.sin(yaw   / 2)
    s2 = math.sin(pitch / 2)
    s3 = math.sin(roll  / 2)

    quaternion = Quaternion()
    quaternion.w = c1 * c2 * c3 - s1 * s2 * s3
    quaternion.x = s1 * s2 * c3 + c1 * c2 * s3
    quaternion.y = s1 * c2 * c3 + c1 * s2 * s3
    quaternion.z = c1 * s2 * c3 + s1 * c2 * s3
    
    return quaternion

class ImuNames:
    _group: str 
    _roll_name: str 
    _pitch_name: str
    _yaw_name: str
    roll: str 
    pitch: str 
    yaw: str

    def __init__(self) -> None:
        self._group = "stabilizer"
        self._roll_name = "roll"
        self._pitch_name = "pitch"
        self._yaw_name = "yaw"

        self.roll = join_group_and_name(self._group, self._roll_name)
        self.pitch = join_group_and_name(self._group, self._pitch_name)
        self.yaw = join_group_and_name(self._group, self._yaw_name)


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
    _imu_names_struct: ImuNames

    
    def __init__(self):
        super().__init__('listener_node')

        self._imu_names_struct = ImuNames()

        # +++ flags        
        self._is_connected = False

        # +++ init drivers
        cflib.crtp.init_drivers()
        
        # +++ create synced cf
        factory = CachedCfFactory(rw_cache="./cache")
        self._synced_cf = factory.construct("radio://0/80/2M/E7E7E7E7E7")

        self.publisher = self.create_publisher(EulerAngle, 'crazyflie/sensor_msgs/imu', 10)

    def _add_standard_callbacks(self):
        self._synced_cf.cf.fully_connected.add_callback(self._fully_connected)
        self._synced_cf.cf.disconnected.add_callback(self._disconnected)
        self._synced_cf.cf.connection_failed.add_callback(self._connection_failed) 

    def _log_imu_callback(self,timestamp, data, logconf):
        """        
        roll (φ): Roll angle
        pitch (θ): Pitch angle
        yaw (ψ): Yaw angle
        """

        log_msg = '[%d][%s]: %s' % (timestamp, logconf.name, data)
        pitch_msg = '[%d][%s]: %s' % (timestamp, logconf.name, data[self._imu_names_struct.pitch])
        # self.get_logger().info(pitch_msg)
        # imu = Imu()
        # imu.orientation = euler_to_quaternion(
        #     roll=data[self._imu_names_struct.roll],
        #     pitch=data[self._imu_names_struct.pitch],
        #     yaw=data[self._imu_names_struct.yaw]
        # )
        euler = EulerAngle()
        euler.roll = data[self._imu_names_struct.roll]
        euler.pitch = data[self._imu_names_struct.pitch]
        euler.yaw = data[self._imu_names_struct.yaw]
        self.publisher.publish(euler)
        # self.publisher.publish(imu)

    def _publish_imu(self):
        """
        Publishes IMU data to a topic.
        """
        float_type = 'float'
        lg_imu = LogConfig(name="Imu", period_in_ms=10)
        lg_imu.add_variable(self._imu_names_struct.roll, float_type)
        lg_imu.add_variable(self._imu_names_struct.pitch, float_type)
        lg_imu.add_variable(self._imu_names_struct.yaw, float_type)

        self._synced_cf.cf.log.add_config(lg_imu)
        lg_imu.data_received_cb.add_callback(self._log_imu_callback)
        lg_imu.start()

    def _publish_variables(self):
        self._publish_imu()

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
        # self.get_logger().debug("fully connected with " + link_uri)
        self._publish_variables()
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

    