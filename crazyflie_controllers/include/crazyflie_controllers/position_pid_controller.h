#pragma once

#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "crazyflie_msgs/msg/position_command.hpp"
#include "crazyflie_msgs/msg/attitude_command.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>

#include "crazyflie_controllers/control_utils/pid.h"

constexpr int CONTROLLER_FREQ = 10;

class PositionPID : public rclcpp::Node {
public:
    PositionPID();

private:
    void _newPositionCommandCallback(const crazyflie_msgs::msg::PositionCommand::SharedPtr command);
    void _newGpsCallback(const geometry_msgs::msg::PointStamped::SharedPtr gps_data);
    void _newImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data);
    void _sendCommandAttitude();

    rclcpp::Subscription<crazyflie_msgs::msg::PositionCommand>::SharedPtr _sub_new_position;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _sub_gps;
    rclcpp::Publisher<crazyflie_msgs::msg::AttitudeCommand>::SharedPtr _pub_attutude_cmd;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _sub_imu;
    rclcpp::TimerBase::SharedPtr _attitude_cmd_timer;

    double _x;
    double _y;
    double _z;
    double _roll;
    double _pitch;
    double _yaw;

    //Rotation quantities of coordinate frames
    tf2::Matrix3x3 R_WBnow; // Rotation going from world frame W to the current body frame B, cannot name it _R_WBnow
    // R_WBnow = R_WBold * R_BoldBnow

    // R_BoldBnow is essentially the Rotation we get from the IMU
    tf2::Matrix3x3 R_BoldBnow; // Rotation going from old body frame to the body frame B now, cannot use prime character
    // R_WBold is the old (the initial Rotation when it starts) Rotation going from world frame to the old body frame B
    tf2::Matrix3x3 R_WBold; // Rotation going from world frame W to the current body frame B, cannot name it _R_WB

    //translation quantities of coordinate frames
    tf2::Vector3 _p_WBnow; // Vector going from the world frame W to the body frame B, essentially holding _x, _y, _z

    //Transformation between coordinate frames
    tf2::Transform X_WBnow; // Transformation from the world frame to the current body frame of the drone
    tf2::Transform X_BnowW; // Transformation from the current body frame of the drone to the world frame

    // vector quantities about the desired position vector
    tf2::Vector3 _p_BnowD; // Vector (measured from body frame B) going from the frame body B to a desired position D
    tf2::Vector3 _p_WD_Bnow; // Vector measured from world frame going from frame B to a desired position D

    // The goal here is to express the desired position measured in the world frame _p_WD_Bnow in the body frame _p_BnowD,
    // so  that we use this vector for the position commands of the drone
    // _p_BnowD = X_BnowW * _p_WD_Bnow

    double _input_yaw;
    double _target_x;
    double _target_y;
    double _target_z;

    PID _pid_x;
    PID _pid_y;
    PID _pid_z;

    rclcpp::Time _old_time;
};