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


    tf2::Matrix3x3 R_WB; // Rotation going from world frame W to the body frame B, cannot name it _R_WB
    tf2::Matrix3x3 R_BW; // Rotation going from body frame B to the world frame W


    // vector quantities
    tf2::Vector3 p_WB_W; // Vector (expressed in body frame W) going from the world frame W to the body frame B

    // vector quantities about the desired position vector
    tf2::Vector3 p_BD_B; // Vector (expressed in body frame B) going from frame B to a desired position D
    tf2::Vector3 p_BD_W; // Vector (expressed in body frame W) going from frame B to a desired position D

    tf2::Vector3 p_WD_W; // Vector (expressed in body frame W) going from world frame to a desired position D

    //Transformation between coordinate frames
    //tf2::Transform X_WB; // Transformation from the world frame to the body frame of the drone, cannot name it _X_WB
    //tf2::Transform X_BW; // Transformation from the body frame of the drone to the world frame

    double _yaw; // The yaw we directly input and give to the atitude controller


    PID _pid_x;
    PID _pid_y;
    PID _pid_z;

    rclcpp::Time _old_time;
};