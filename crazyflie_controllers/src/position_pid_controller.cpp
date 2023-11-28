#include <tf2/LinearMath/Matrix3x3.h>
#include "crazyflie_controllers/position_pid_controller.h"

PositionPID::PositionPID() : 
    Node("attitude_pid_controller"), _input_yaw(0.0), _target_x(0.0), _target_y(0.0), _target_z(0.0),
        _pid_x(0.01, 0, 0.04, true, 0.1), _pid_y(0.01, 0, 0.04, true, 0.1), _pid_z(8, 1 , 10, true, 0.1)
{
        _sub_new_position = this->create_subscription<crazyflie_msgs::msg::PositionCommand>(
            "/crazyflie/pid/position_controller",
            10,
            std::bind(&PositionPID::_newPositionCommandCallback, this, std::placeholders::_1));

        _sub_gps = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/crazyflie/gps",
            10,
            std::bind(&PositionPID::_newGpsCallback, this, std::placeholders::_1));
        _sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
            "/crazyflie/imu",
            10,
            std::bind(&PositionPID::_newImuCallback, this, std::placeholders::_1));

        _pub_attutude_cmd = this->create_publisher<crazyflie_msgs::msg::AttitudeCommand>(
            "/crazyflie/pid/attitude_controller",
            10);

        _attitude_cmd_timer = this->create_wall_timer(
            std::chrono::milliseconds(1000 / CONTROLLER_FREQ),
            std::bind(&PositionPID::_sendCommandAttitude, this));

        _old_time = now();
  
}

void PositionPID::_newPositionCommandCallback(const crazyflie_msgs::msg::PositionCommand::SharedPtr command)
{
    _target_x = command->x;
    _target_y = command->y;
    _target_z = command->z;
    _yaw = command->yaw;
    RCLCPP_INFO(this->get_logger(), "target updated, x: %f, y: %f, z= %f", _target_x, _target_y, _target_z);
}

void PositionPID::_newGpsCallback(const geometry_msgs::msg::PointStamped::SharedPtr gps_data)
{
    _x = gps_data->point.x;
    _y = gps_data->point.y;
    _z = gps_data->point.z;

}

void PositionPID::_newImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data) {
    tf2::Quaternion quaternion(
            imu_data->orientation.x,
            imu_data->orientation.y,
            imu_data->orientation.z,
            imu_data->orientation.w);

    tf2::Matrix3x3 mat(quaternion);
    mat.getRPY(_roll, _pitch, _yaw);
    _body_orientation = mat;

    RCLCPP_DEBUG(this->get_logger(), "imu data received! [r: %f, p: %f, y: %f]", _roll, _pitch, _yaw);
}

void PositionPID::_sendCommandAttitude()
{
    rclcpp::Duration dt = now() - _old_time; 
    
    double pid_x = _pid_x.getCommand(_x, _target_x, dt);
    double pid_y = _pid_y.getCommand(_y, _target_y, dt);
    double pid_z = _pid_z.getCommand(_z, _target_z, dt);


    // first map a difference in x-, y-, z-direction direction to corresponding gains in attitude gains (and thrust)
    tf2::Vector3 desired_action(pid_x, - pid_y, pid_z); // For the explanation of the - pid_y consult the Readme

    // map the desired action to body frame of drone by using the inverse (the transpose for rotation matrices) of the
    // body frame rotation matrix
    tf2::Vector3 mapped_action = _body_orientation * desired_action;

    auto msg = std::make_unique<crazyflie_msgs::msg::AttitudeCommand>();

    msg->pitch = mapped_action.x();
    msg->roll = mapped_action.y();
    msg->thurst = mapped_action.z();
    msg->yaw = _yaw;

    _pub_attutude_cmd->publish(std::move(msg));

    _old_time = now();
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionPID>());
    rclcpp::shutdown();
    return 0;
}
