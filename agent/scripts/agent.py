#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import Image
from crazyflie_msgs.msg import MotorVel
import cv2
import numpy as np
from cv_bridge import CvBridge
from deepbots.supervisor.controllers.robot_supervisor_env import RobotSupervisorEnv
import rospy

from gym.spaces import Box, Discrete
import numpy as np

class Drone(RobotSupervisorEnv, Node):
    def __init__(self):
        super().__init__()

        self.resolution = (128, 128, 3) # TODO correct resolution
        self.observation_space = Box(low=0, high=255, shape=self.resolution, dtype='uint8')
        self.action_space = Box(low=0, high=100, shape=(4,), dtype='float32')

        self.motors = []
        for motor_name in ['m1_motor', 'm2_motor', 'm3_motor', 'm4_motor']:
            motor = self.getDevice(motor_name)
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0) # maybe more to not fall?
            self.motors.append(motor)

        # TODO: get sensors the same way as motors
        self.camera = rospy.Subscriber('/crazyflie/camera', Image, self.camera_callback, queue_size=1)
        self.current_image = None

        self.steps_per_episode = 1000 # TODO: get optimal value
        self.episode_score = 0
        self.episode_score_list = []
    
    def camera_callback(self, img):
        image = img.data
        # convert from ros image to cv2 image
        image = self.bridge.imgmsg_to_cv2(img, desired_encoding='passthrough')
        # convert to grayscale
        image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        # resize to (128, 128, 3)
        image = cv2.resize(image, self.resolution)
        self.current_image = image

    
    def get_observations(self):
        return self.current_image # TODO not sure about implementation
    
    def get_default_observation(self):
        return np.zeros((128, 128, 3)) # DUMMY - TODO

    def get_reward(self, action=None):
        return 1 # DUMMY - TODO
    
    def is_done(self):
        return True if self.episode_score > 100 else False # DUMMY - TODO
    
    def solved(self):
        return False # DUMMY - TODO
    
    def get_info(self):
        return None

    def render(self, mode='human'):
        pass

    def apply_action(self, action):
        for i in range(len(self.motors)):
            self.wheels[i].setPosition(float('inf'))
            self.wheels[i].setVelocity(action[i])
        
# class TestController(Node):
    
#     def __init__(self):
#         super().__init__('test_controller')
#         self._cmd_motor_vel_pub = self.create_publisher(MotorVel, 'crazyflie/cmd_motor_vel', 10)
#         self._first_time = True

#         timer_period = 2  
#         self._timer = self.create_timer(timer_period, self.timer_callback)

#     def timer_callback(self):
#         if self._first_time:
#             propeller_vel = 500.0
#             self.get_logger().info('Publishing cmd_motor_vel for the first time')
#             self._first_time = False
#         else:
#             propeller_vel = 53.0
        
#         msg = MotorVel()
#         msg.m1 =   propeller_vel
#         msg.m2 =   propeller_vel
#         msg.m3 =   propeller_vel
#         msg.m4 =   propeller_vel
#         self._cmd_motor_vel_pub.publish(msg)
        
class Agent():
    def __init__(self) -> None:
        pass


class Trainer():
    def __init__(self):
        self.env = Drone()
        self.agent = None
        self.solved = False
        self.episode_count = 0
        self.episode_limit = 2000

        # TODO the rest https://github.com/aidudezzz/deepbots-tutorials/blob/master/robotSupervisorSchemeTutorial/full_project/controllers/robot_supervisor_controller/robot_supervisor_controller.py
        
    

def main(args=None):
    rclpy.init(args=args)
    target_publisher = ApriltagDetector()
    rclpy.spin(target_publisher)

if __name__ == '__main__':
    main()
