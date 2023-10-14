#include "rclcpp/rclcpp.hpp"
#include "crazyflie_msgs/msg/attitude_command.hpp"
#include "crazyflie_msgs/msg/motor_vel.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <tf2/transform_datatypes.h>

constexpr double GRAVITY_COMPENSATION = 50.0;
constexpr int CONTROLLER_FREQ = 50;

class PID {
public:
    PID(double kp = 0, double ki = 0, double kd = 0)
        : _kp(kp), _ki(ki), _kd(kd), _distance_previous_error(0), _count(0) {}

    double getCommand(double value, double target) {
        double distance_error = target - value;
        double pid_P = _kp * distance_error;
        std::cout << _count << " " << distance_error << std::endl;
        _count++;

        double pid_I = 0;

        double dist_difference = (distance_error - _distance_previous_error) * CONTROLLER_FREQ;
        double pid_D = _kd * dist_difference;
        _distance_previous_error = distance_error;

        double command = pid_P + pid_I + pid_D;
        return command;
    }

private:
    double _kp;
    double _ki;
    double _kd;
    double _distance_previous_error;
    int _count;
};

class AttitudePID : public rclcpp::Node {
public:
    AttitudePID()
        : Node("attitude_pid_controller"), _target_z(2), _target_roll(0), _target_pitch(0), _target_yaw(0) {
        _sub_new_position = this->create_subscription<crazyflie_msgs::msg::AttitudeCommand>(
            "/crazyflie/pid/attitude_controller",
            10,
            std::bind(&AttitudePID::_newCommandCallback, this, std::placeholders::_1));

        _sub_gps = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/crazyflie/gps",
            10,
            std::bind(&AttitudePID::_newGpsCallback, this, std::placeholders::_1));

        _sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
            "/crazyflie/imu",
            10,
            std::bind(&AttitudePID::_newImuCallback, this, std::placeholders::_1));

        _pub_motor_vel = this->create_publisher<crazyflie_msgs::msg::MotorVel>(
            "/crazyflie/cmd_motor_vel",
            10);

        _cmd_motor_timer = this->create_wall_timer(
            std::chrono::milliseconds(1000 / CONTROLLER_FREQ),
            std::bind(&AttitudePID::_sendCmdMotor, this));

        _pid_z = PID(10, 50, 5);
        _pid_roll = PID(2, 0, 30);
        _pid_pitch = PID(2, 0, 30);
    }

private:
    void _newCommandCallback(const crazyflie_msgs::msg::AttitudeCommand::SharedPtr command) {
        _target_pitch = command->pitch;
        _target_roll = command->roll;
        _target_yaw = command->yaw;
        _target_z = command->thurst;
    }

    void _newImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data) {
        tf2::Quaternion quaternion(
            imu_data->orientation.x,
            imu_data->orientation.y,
            imu_data->orientation.z,
            imu_data->orientation.w);

        tf2::Matrix3x3 mat(quaternion);
        mat.getRPY(_roll, _pitch, _yaw);
        RCLCPP_INFO(this->get_logger(), "imu data received! [r: %f, p: %f, y: %f]", _roll, _pitch, _yaw);
    }

    void _newGpsCallback(const geometry_msgs::msg::PointStamped::SharedPtr gps_data) {
        _x = gps_data->point.x;
        _y = gps_data->point.y;
        _z = gps_data->point.z;
        RCLCPP_INFO(this->get_logger(), "gps data received! [x: %f, y: %f, z: %f]", _x, _y, _z);
    }

    void _sendCmdMotor() {
        double throttle = _pid_z.getCommand(_z, _target_z);
        // double roll = _pid_roll.getCommand(_roll, _target_roll);
        // double pitch = _pid_pitch.getCommand(_pitch, _target_pitch);

        auto msg = std::make_unique<crazyflie_msgs::msg::MotorVel>();
        msg->m1 = GRAVITY_COMPENSATION + throttle; // - roll
        msg->m2 = GRAVITY_COMPENSATION + throttle; // - roll
        msg->m3 = GRAVITY_COMPENSATION + throttle; // + roll
        msg->m4 = GRAVITY_COMPENSATION + throttle; // + roll
        _pub_motor_vel->publish(std::move(msg));
    }

    rclcpp::Subscription<crazyflie_msgs::msg::AttitudeCommand>::SharedPtr _sub_new_position;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr _sub_gps;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _sub_imu;
    rclcpp::Publisher<crazyflie_msgs::msg::MotorVel>::SharedPtr _pub_motor_vel;
    rclcpp::TimerBase::SharedPtr _cmd_motor_timer;

    PID _pid_z;
    PID _pid_roll;
    PID _pid_pitch;

    double _x;
    double _y;
    double _z;
    double _roll;
    double _pitch;
    double _yaw;

    double _target_z;
    double _target_roll;
    double _target_pitch;
    double _target_yaw;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AttitudePID>());
    rclcpp::shutdown();
    return 0;
}
