import rclpy
from crazyflie_msgs.msg import MotorVel


class CrazyflieWebotsDriver:

    def init(self, webots_node, properties):

        self.__robot = webots_node.robot
        rclpy.init(args=None)

        self.__node = rclpy.create_node('my_robot_driver')
        self.__node.create_subscription(MotorVel, 'crazyflie/cmd_motor_vel', self.__cmd_motor_vel_callback, 1)

        self.__node.get_logger().warning("hello")

        self.__m1_motor = self.__robot.getDevice("m1_motor")
        self.__m1_motor.setPosition(float('inf'))

        self.__m2_motor = self.__robot.getDevice("m2_motor")
        self.__m2_motor.setPosition(float('inf'))

        self.__m3_motor = self.__robot.getDevice("m3_motor")
        self.__m3_motor.setPosition(float('inf'))

        self.__m4_motor = self.__robot.getDevice("m4_motor")
        self.__m4_motor.setPosition(float('inf'))

        self.__target_motor_vel = MotorVel()

    def __cmd_motor_vel_callback(self, motor_vel: MotorVel):
        self.__target_motor_vel = motor_vel

    def step(self):

        self.__m1_motor.setVelocity(- self.__target_motor_vel.m1)
        self.__m2_motor.setVelocity(self.__target_motor_vel.m2)
        self.__m3_motor.setVelocity(- self.__target_motor_vel.m3)
        self.__m4_motor.setVelocity(self.__target_motor_vel.m4)

        rclpy.spin_once(self.__node, timeout_sec=0)
