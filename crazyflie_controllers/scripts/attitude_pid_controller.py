#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from dataclasses import dataclass
from crazyflie_msgs.msg import AttitudeCommand
from crazyflie_msgs.msg import MotorVel
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Imu
from tf_transformations import euler_from_quaternion

GRAVITY_COMPENSATION = 55.0
CONTROLLER_FREQ = 50
    
class PID:

    def __init__(self,kp=0,ki=0,kd=0) -> None:
        self._kp = kp
        self._ki = kd
        self._kd = kd
        self._distance_previous_error = 0
        self._count = 0

    def get_command(self, value, target):
        
        distance_error = target - value
        pid_P = self._kp * distance_error
        print(""+str(self._count) + " " +str(distance_error))
        self._count += 1
        
        pid_I = 0
        
        dist_difference = (distance_error - self._distance_previous_error) * CONTROLLER_FREQ
        pid_D = self._kd * dist_difference
        self._distance_previous_error = distance_error

        command = pid_P + pid_I + pid_D
        return command




class AttitudePID(Node):
    
    def __init__(self) -> None:
        
        super().__init__('attitude_pid_controller')

        self._x = 0
        self._y = 0
        self._z = 0
        self._roll = 0
        self._pitch = 0
        self._yaw = 0
    
        self._sub_new_position = self.create_subscription(
                AttitudeCommand ,"/crazyflie/pid/attitude_controller", self._new_command_callback,10)
        self._sub_gps = self.create_subscription(
                PointStamped, "/crazyflie/gps", self._new_gps_callback, 10 )
        self._sub_imu = self.create_subscription( Imu,"/crazyflie/imu", self._new_imu_callback, 10)
        self._pub_motor_vel = self.create_publisher(
                MotorVel, "/crazyflie/cmd_motor_vel", 10 )
        
        self._cmd_motor_timer = self.create_timer(1/CONTROLLER_FREQ, self._send_cmd_motor)
        
        self._pid_z = PID(kp=0.3,ki=0,kd=50)
        self._pid_roll = PID(kp=2,ki=0,kd=30)
        self._pid_pitch = PID(kp=2,ki=0,kd=30)
        
        self._target_z = 2
        self._target_roll = 0
        self._target_pitch = 0
        self._target_yaw = 0
        


    
    def _new_command_callback(self, command : AttitudeCommand):
        
        self._target_pitch = command.pitch
        self._target_roll = command.roll
        self._target_yaw = command.yaw
        self._target_z = command.thurst


    def _new_imu_callback(self, imu_data : Imu):

        quaternion = [
            imu_data.orientation.x,
            imu_data.orientation.y,
            imu_data.orientation.z,
            imu_data.orientation.w]
        (self._roll, self._pitch, self._yaw) = euler_from_quaternion(quaternion)
        self.get_logger().debug(f"imu data received! \
            [r: {self._roll}, p: {self._pitch}, y: {self._yaw}]")
        

    def _new_gps_callback(self, gps_data : PointStamped):
        
        self._x = gps_data.point.x
        self._y = gps_data.point.y
        self._z = gps_data.point.z
        self.get_logger().debug(f"gps data received! \
            [x: {gps_data.point.x}, y: {gps_data.point.y}, z: {gps_data.point.z}]")
        
        

             

    def _send_cmd_motor(self):

        throttle = self._pid_z.get_command(value=self._z, target=self._target_z) 
        #roll = self._pid_roll.get_command(value=self._roll, target=self._target_roll)
        #pitch = self._pid_pitch.get_command(value=self._pitch, target=self._target_pitch)

        msg = MotorVel()
        msg.m1 = GRAVITY_COMPENSATION + throttle #- roll
        msg.m2 = GRAVITY_COMPENSATION + throttle #- roll
        msg.m3 = GRAVITY_COMPENSATION + throttle #+ roll
        msg.m4 = GRAVITY_COMPENSATION + throttle #+ roll
        self._pub_motor_vel.publish(msg)
   

def main(args=None):
    rclpy.init(args=args)
    node = AttitudePID()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    

    
