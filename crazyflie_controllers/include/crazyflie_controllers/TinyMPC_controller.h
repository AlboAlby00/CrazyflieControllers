#pragma once

#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "crazyflie_msgs/msg/position_command.hpp"
#include "crazyflie_msgs/msg/attitude_command.hpp"
#include "crazyflie_msgs/msg/motor_vel.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mutex>
#include <chrono>

#include "crazyflie_controllers/control_utils/pid.h"
#include "crazyflie_controllers/control_utils/ModelPredictiveController.h"
#include "../src/tinympc/admm.hpp"
#include "../../src/tinympc/problem_data/quadrotor_20hz_params.hpp"


constexpr int CONTROLLER_FREQ = 10;

class PositionMPC : public rclcpp::Node {
public:
    PositionMPC();

private:
    void _newPositionCommandCallback(const crazyflie_msgs::msg::PositionCommand::SharedPtr command);
    void _newGpsCallback(const geometry_msgs::msg::PointStamped::SharedPtr gps_data);
    void _newImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data);
    void _sendCommandAttitude();

    rclcpp::Subscription<crazyflie_msgs::msg::PositionCommand>::SharedPtr _sub_new_position;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _sub_gps;
    rclcpp::Publisher<crazyflie_msgs::msg::MotorVel>::SharedPtr _pub_motor_vel;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _sub_imu;
    rclcpp::TimerBase::SharedPtr _attitude_cmd_timer;

    tf2::Matrix3x3 R_WB; // Rotation going from world frame W to the body frame B, cannot name it _R_WB
    tf2::Matrix3x3 R_BW; // Rotation going from body frame B to the world frame W


    // position vector quantities
    tf2::Vector3 p_WB_W; // Vector (expressed in body frame W) going from the world frame W to the body frame B
    tf2::Vector3 p_WB_W_prev; // Previous vector (expressed in body frame W) going from world frame to the body frame B

    // position vector quantities about the desired position vector
    tf2::Vector3 p_BD_B; // Vector (expressed in body frame B) going from frame B to a desired position D
    tf2::Vector3 p_BD_W; // Vector (expressed in body frame W) going from frame B to a desired position D

    tf2::Vector3 p_WD_W; // Vector (expressed in body frame W) going from world frame to a desired position D


    // positional velocity quantities
    tf2::Vector3 v_WB; //Point B's (The body frame B of the drone) translational velocity in frame W

    // angular velocity quantities
    tf2::Vector3 omega_WB; //Frame B's angular velocity in frame W

    //Transformation between coordinate frames
    //tf2::Transform X_WB; // Transformation from the world frame to the body frame of the drone, cannot name it _X_WB
    //tf2::Transform X_BW; // Transformation from the body frame of the drone to the world frame

    double _yaw; // The yaw we directly input and give to the atitude controller

    rclcpp::Time _prev_time;
    rclcpp::Time _prev_time_position;
    bool _is_prev_time_position_set;

    ModelPredictiveController _mpc;
    std::mutex _mpc_mutex;


    unsigned int _f, _v;
    unsigned int _timeSteps;

    const double _Ix = 0.0000166;  // Moment of inertia around p_WB_W_x-axis, source: Julian Förster's ETH Bachelor Thesis
    const double _Iy = 0.0000167;  // Moment of inertia around p_WB_W_y-axis, source: Julian Förster's ETH Bachelor Thesis
    const double _Iz = 0.00000293;  // Moment of inertia around p_WB_W_z-axis, source: Julian Förster's ETH Bachelor Thesis
    const double _mass = 0.029;  // Mass of the quadrotor, source: Julian Förster's ETH Bachelor Thesis
    const double _g = 9.81;     // Acceleration due to gravity

    void InitializeMPC();

    bool _desiredControlSetByCallback = false;

    TinyCache cache;
    TinyWorkspace work;
    TinySettings settings;
    TinySolver solver{};

    tiny_VectorNx x0, x1; // current and next simulation states
};