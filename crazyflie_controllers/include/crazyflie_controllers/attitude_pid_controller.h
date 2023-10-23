#pragma once

#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "crazyflie_msgs/msg/attitude_command.hpp"
#include "crazyflie_msgs/msg/pid_tuner.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "crazyflie_msgs/msg/motor_vel.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/transform_datatypes.h>

#include "crazyflie_controllers/control_utils/pid.h"

constexpr double GRAVITY_COMPENSATION = 50.0;
constexpr int CONTROLLER_FREQ = 30;

class AttitudePID : public rclcpp::Node {
public:
    AttitudePID();

private:
    void _newCommandCallback(const crazyflie_msgs::msg::AttitudeCommand::SharedPtr command);
    void _newImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data);
    void _newGpsCallback(const geometry_msgs::msg::PointStamped::SharedPtr gps_data);
    void _sendCmdMotor();

    rclcpp::Subscription<crazyflie_msgs::msg::AttitudeCommand>::SharedPtr _sub_new_position;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _sub_gps;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _sub_imu;
    rclcpp::Publisher<crazyflie_msgs::msg::MotorVel>::SharedPtr _pub_motor_vel;
    rclcpp::TimerBase::SharedPtr _cmd_motor_timer;

    double _x;
    double _y;
    double _z;
    double _roll;
    double _pitch;
    double _yaw;

    double _input_thrust;
    double _target_roll;
    double _target_pitch;
    double _target_yaw;

    Angular_PID _pid_roll;
    Angular_PID _pid_pitch;
    Angular_PID _pid_yaw;

    rclcpp::Time _old_time;
};