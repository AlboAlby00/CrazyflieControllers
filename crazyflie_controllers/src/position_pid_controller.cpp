#include <tf2/LinearMath/Matrix3x3.h>
#include "crazyflie_controllers/position_pid_controller.h"

PositionPID::PositionPID() : 
    Node("position_pid_controller"), _yaw(0.0), _target_x(0.0), _target_y(0.0), _target_z(0.0),
        _pid_x(0.01, 0, 0.04, false, true, 0.1), _pid_y(0.01, 0, 0.04, false, true, 0.1), _pid_z(8, 1, 10, false, true, 0.1)
{
        this->declare_parameter("state_est", "camera");
        std::string state_est_param = this->get_parameter("state_est").as_string();
        RCLCPP_INFO(this->get_logger(), "state: %s", state_est_param.c_str());
        std::string state_est_topic = state_est_param == "camera" ? "/crazyflie/camera_position" : "/crazyflie/gps";

        _sub_new_position = this->create_subscription<crazyflie_msgs::msg::PositionCommand>(
            "/crazyflie/pid/position_controller",
            10,
            std::bind(&PositionPID::_newPositionCommandCallback, this, std::placeholders::_1));

        _sub_state = this->create_subscription<geometry_msgs::msg::PointStamped>(
            state_est_topic,
            10,
            std::bind(&PositionPID::_newStateCallback, this, std::placeholders::_1));

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
    // Vector (expressed in body frame B) going from frame B to a desired position D, also p_BD_B
    p_WD_W = tf2::Vector3(command->x, command->y, command->z);

    p_BD_W = p_WD_W - p_WB_W;

    R_BW = R_WB.transpose();

    // The goal here is to express the Vector p_BD_W (expressed in body frame W) going from frame B to a desired
    // position D in the Vector in the body frame B, i.e. p_BD_B.
    // Then we use this vector for the position control of the drone for correct PID gains calculation.
    // We have to update p_BD_B, because p_WD_W changes over time. This is done in the GpsCallback
    p_BD_B = R_BW * p_BD_W;

    // Step 4: Set the target positions in the drone's local coordinate system.
    _target_x = p_BD_B.x();
    _target_y = p_BD_B.y();
    _target_z = p_BD_B.z();
    
    RCLCPP_DEBUG(this->get_logger(),
                "target updated, p_BD_B.x(): %f, p_BD_B.y(): %f, p_BD_B.z() = %f",
                p_BD_B.x(), p_BD_B.y(), p_BD_B.z());
}

void PositionPID::_newStateCallback(const geometry_msgs::msg::PointStamped::SharedPtr gps_data)
{
    // Vector (expressed in body frame W) going from the world frame W to the body frame B, time variant.
    p_WB_W = tf2::Vector3(gps_data->point.x, gps_data->point.y, gps_data->point.z);

    p_BD_W = p_WD_W - p_WB_W;

    R_BW = R_WB.transpose();

    // The goal here is to express the Vector p_BD_W (expressed in body frame W) going from frame B to a desired
    // position D in the Vector in the body frame B, i.e. p_BD_B.
    // Then we use this vector for the position control of the drone for correct PID gains calculation.
    // We have to update p_BD_B, because p_WD_W changes over time. This is done in the GpsCallback
    p_BD_B = R_BW * p_BD_W;



    RCLCPP_DEBUG(this->get_logger(),
                 "GPS updated, p_WB.x: %f, p_WB.y(): %f, p_WB.z = %f", p_WB_W.x(), p_WB_W.y(), p_WB_W.z());
}

void PositionPID::_newImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data) {
    tf2::Quaternion quaternion(
            imu_data->orientation.x,
            imu_data->orientation.y,
            imu_data->orientation.z,
            imu_data->orientation.w);

    // Rotation going from world frame W to the body frame B, cannot name it _R_WB
    R_WB = tf2::Matrix3x3(quaternion);


    double roll, pitch, yaw;
    R_WB.getRPY(roll, pitch, yaw);

    RCLCPP_DEBUG(this->get_logger(),
                 "imu data received! [r: %f, p: %f, y: %f]",
                 roll, pitch, yaw);
}

void PositionPID::_sendCommandAttitude()
{
    rclcpp::Duration dt = now() - _old_time; 
    
    //double pid_x = _pid_x.getCommand(p_WB_W.x(), p_WB_W.x(), dt);
    //double pid_y = _pid_y.getCommand(p_WB_W.y(), p_WB_W.y(), dt);
    //double pid_z = _pid_z.getCommand(p_WB_W.z(), p_WB_W.z(), dt);

    double pid_x = _pid_x.getCommand(0, p_BD_B.x(), dt);
    double pid_y = _pid_y.getCommand(0, p_BD_B.y(), dt);
    double pid_z = _pid_z.getCommand(0, p_BD_B.z(), dt);

    auto msg = std::make_unique<crazyflie_msgs::msg::AttitudeCommand>();

    msg->pitch = pid_x;

    // The unique minus here results from the fact, that if we want to go into +y direction, we need to
    // have negative roll angle change
    msg->roll = - pid_y;
    msg->thurst = pid_z;
    msg->yaw = _yaw;

    _pub_attutude_cmd->publish(std::move(msg));

    _old_time = now();

    /*RCLCPP_INFO(this->get_logger(),
                 "Sent out Commands! [pitch = pid_x: %f, roll = -pid_y: %f, thrust = pid_z: %f]",
                 pid_x, -pid_y, pid_z);*/
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionPID>());
    rclcpp::shutdown();
    return 0;
}
