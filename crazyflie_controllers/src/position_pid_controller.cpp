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
            "/crazyflie/camera_position",
            10,
            std::bind(&PositionPID::_newGpsCallback, this, std::placeholders::_1));

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

void PositionPID::_sendCommandAttitude()
{
    rclcpp::Duration dt = now() - _old_time; 
    
    double pid_x = _pid_x.getCommand(_x, _target_x, dt);
    double pid_y = _pid_y.getCommand(_y, _target_y, dt);
    double pid_z = _pid_z.getCommand(_z, _target_z, dt);

    auto msg = std::make_unique<crazyflie_msgs::msg::AttitudeCommand>();

    msg->pitch = pid_x;
    msg->roll = - pid_y;
    msg->thurst = pid_z;
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
