#pragma once

#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "crazyflie_msgs/msg/position_command.hpp"
#include "crazyflie_msgs/msg/attitude_command.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2/LinearMath/Matrix3x3.h>
//#include <acado_toolkit.hpp>
//#include <acado_gnuplot.hpp>
//#include <acado_optimal_control.hpp>
#include <acado/acado_toolkit.hpp>
#include <acado/acado_gnuplot.hpp>
#include <acado/acado_optimal_control.hpp>
#include <acado/utils/acado_utils.hpp>
using namespace ACADO;

#include "crazyflie_controllers/control_utils/pid.h"

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
    rclcpp::Publisher<crazyflie_msgs::msg::AttitudeCommand>::SharedPtr _pub_attutude_cmd;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _sub_imu;
    rclcpp::TimerBase::SharedPtr _attitude_cmd_timer;

    double _x;
    double _y;
    double _z;
    double _roll;
    double _pitch;
    double _yaw;
    tf2::Matrix3x3 _body_orientation;

    double _input_yaw;
    double _target_x;
    double _target_y;
    double _target_z;

    rclcpp::Time _old_time;
};