#pragma once

#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "crazyflie_msgs/msg/position_command.hpp"
#include "crazyflie_msgs/msg/attitude_command.hpp"
#include "crazyflie_msgs/msg/motor_vel.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <mutex>
#include <chrono>

#include "crazyflie_controllers/control_utils/pid.h"
#include "crazyflie_controllers/control_utils/ModelPredictiveController.h"
#include "../src/tinympc/admm.hpp"
//#include "../../src/tinympc/problem_data/quadrotor_20hz_params.hpp"
#include "../../src/tinympc/problem_data/quadrotor_100hz_params.hpp"
#include "../../src/tinympc/types.hpp"


constexpr int CONTROLLER_FREQ = 10;
constexpr double GRAVITY_COMPENSATION = 54.22; //54.22, 55.3681225 is the threshold thrust value which lets the drone just levitate from ground very slowly
// 54.22 is the value which lets the drone stay on the ground at the beginning of the simulation bounce + 1.1487125 in thrust lets it levitate slowly,
// but setting this to the comnbined value 54.22 + 1.1487125 = 55.3687125 let's it get off ground very fast at the beginning

class TinyMPC : public rclcpp::Node {
public:
    TinyMPC();

private:
    void _newPositionCommandCallback(const crazyflie_msgs::msg::PositionCommand::SharedPtr command);

    void _newGpsCallback(const geometry_msgs::msg::PointStamped::SharedPtr gps_data);

    void _newGpsSpeedCallback(const geometry_msgs::msg::Vector3 gps_speedVec);

    void _newImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data);

    void _sendCommand();

    rclcpp::Subscription<crazyflie_msgs::msg::PositionCommand>::SharedPtr _sub_new_position;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _sub_gps;
    rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr _sub_gps_speed;
    rclcpp::Publisher<crazyflie_msgs::msg::MotorVel>::SharedPtr _pub_motor_vel;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _sub_imu;
    rclcpp::TimerBase::SharedPtr _cmd_timer;

    tf2::Matrix3x3 R_WB; // Rotation going from world frame W to the body frame B, cannot name it _R_WB
    tf2::Matrix3x3 R_BW; // Rotation going from body frame B to the world frame W


    // position vector quantities
    tf2::Vector3 p_WB_W; // Vector (expressed in body frame W) going from the world frame W to the body frame B

    // position vector quantities about the desired position vector
    tf2::Vector3 p_BD_B; // Vector (expressed in body frame B) going from frame B to a desired position D
    tf2::Vector3 p_BD_W; // Vector (expressed in body frame W) going from frame B to a desired position D

    tf2::Vector3 p_WD_W; // Vector (expressed in body frame W) going from world frame to a desired position D


    // positional velocity quantities
    tf2::Vector3 v_WB; //Point B's (The body frame B of the drone) translational velocity in frame W

    // angular velocity quantities
    tf2::Vector3 omega_WB; //Frame B's angular velocity in frame W

    tf2::Quaternion quaternion_WB;
    Eigen::Vector3d rodriguez_param_WB;

    void initializeMPC();


    const double _Ix = 0.0000166;  // Moment of inertia around p_WB_W_x-axis, source: Julian Förster's ETH Bachelor Thesis
    const double _Iy = 0.0000167;  // Moment of inertia around p_WB_W_y-axis, source: Julian Förster's ETH Bachelor Thesis
    const double _Iz = 0.00000293;  // Moment of inertia around p_WB_W_z-axis, source: Julian Förster's ETH Bachelor Thesis
    const double _mass = 0.029;  // Mass of the quadrotor, source: Julian Förster's ETH Bachelor Thesis
    const double _g = 9.81;     // Acceleration due to gravity

    TinyCache cache;
    TinyWorkspace work;
    TinySettings settings;

    tiny_VectorNx x0;
    tiny_VectorNx x_ref;
};