#include <tf2/LinearMath/Matrix3x3.h>
#include "crazyflie_controllers/position_pid_controller.h"

PositionPID::PositionPID() : 
    Node("position_pid_controller"), _input_yaw(0.0), _target_x(0.0), _target_y(0.0), _target_z(0.0),
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

        R_WBnow.setIdentity();

        R_BoldBnow.setIdentity();
        R_WBold.setIdentity();
        X_WBnow.setIdentity();

}

void PositionPID::_newPositionCommandCallback(const crazyflie_msgs::msg::PositionCommand::SharedPtr command)
{
    //Assumption Vector measured from world frame going from frame B to a desired position D
    _p_WD_Bnow = tf2::Vector3(command->x, command->y, command->z);

    // X_BnowW is the Transformation from the current body frame of the drone to the world frame
    X_BnowW = X_WBnow.inverse();

    // The goal here is to express the desired position measured in the world frame _p_WD_Bnow in the body frame _p_BnowD,
    // so  that we use this vector for the position commands of the drone
    _p_BnowD = X_BnowW * _p_WD_Bnow;

    // Step 4: Set the target positions in the drone's local coordinate system.
    _target_x = _p_BnowD.x();
    _target_y = _p_BnowD.y();
    _target_z = _p_BnowD.z();
    _yaw = command->yaw;
    RCLCPP_INFO(this->get_logger(),
                "target updated, _target_x = _p_BnowD.x: %f, _target_y = _p_BnowD.y: %f, _target_z = _p_BnowD.z = %f",
                _target_x, _target_y, _target_z);
}

void PositionPID::_newGpsCallback(const geometry_msgs::msg::PointStamped::SharedPtr gps_data)
{
    // Vector going from the world frame W to the body frame B, essentially holding _x, _y, _z
    _p_WBnow = tf2::Vector3(gps_data->point.x, gps_data->point.y, gps_data->point.z);

    _x = _p_WBnow.x();
    _y = _p_WBnow.y();
    _z = _p_WBnow.z();

    X_WBnow.setOrigin(_p_WBnow);


    RCLCPP_INFO(this->get_logger(),
                 "GPS updated, _x = _p_WBnow.x: %f, _y = _p_WBnow.y(): %f, _z = _p_WBnow.z = %f", _x, _y, _z);
}

void PositionPID::_newImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data) {
    tf2::Quaternion quaternion(
            imu_data->orientation.x,
            imu_data->orientation.y,
            imu_data->orientation.z,
            imu_data->orientation.w);

    // Update the previous Rotation R_WBold to be the R_WBnow from last time
    R_WBold = R_WBnow;

    // Rotation going from old body frame to the current body frame Bnow. Informatiion gained from IMU
    R_BoldBnow = tf2::Matrix3x3(quaternion); // actually a member, but cannot name it _R_WB

    // Combine the previous Rotation with the new Rotation gained from the IMU
    R_WBnow = R_WBold * R_BoldBnow;

    X_WBnow.setBasis(R_WBnow);

    R_WBnow.getRPY(_roll, _pitch, _yaw);

    RCLCPP_INFO(this->get_logger(),
                 "imu data received! [r: %f, p: %f, y: %f]",
                 _roll, _pitch, _yaw);
}

void PositionPID::_sendCommandAttitude()
{
    rclcpp::Duration dt = now() - _old_time; 
    
    double pid_x = _pid_x.getCommand(_p_WBnow.x(), _p_BnowD.x(), dt);
    double pid_y = _pid_y.getCommand(_p_WBnow.y(), _p_BnowD.y(), dt);
    double pid_z = _pid_z.getCommand(_p_WBnow.z(), _p_BnowD.z(), dt);


    /* Made obsolete with my MIT notation
    // first map a difference in x-, y-, z-direction direction to corresponding gains in attitude gains (and thrust)
    tf2::Vector3 desired_action(pid_x, - pid_y, pid_z); // For the explanation of the - pid_y consult the Readme

    // map the desired action to body frame of drone by using the inverse (the transpose for rotation matrices) of the
    // body frame rotation matrix
    tf2::Vector3 mapped_action = R_WB * desired_action;
     */

    auto msg = std::make_unique<crazyflie_msgs::msg::AttitudeCommand>();

    msg->pitch = pid_x;
    msg->roll = -pid_y;
    msg->thurst = pid_z;
    msg->yaw = _yaw;

    _pub_attutude_cmd->publish(std::move(msg));

    _old_time = now();

    RCLCPP_INFO(this->get_logger(),
                 "Sent out Commands! [pitch = pid_x: %f, roll = -pid_y: %f, thurst = pid_z: %f]",
                 pid_x, -pid_y, pid_z);
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionPID>());
    rclcpp::shutdown();
    return 0;
}
