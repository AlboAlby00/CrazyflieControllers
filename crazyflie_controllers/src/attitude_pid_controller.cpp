#include "crazyflie_controllers/attitude_pid_controller.h"

AttitudePID::AttitudePID()
    : Node("attitude_pid_controller"), _input_thrust(1.0), _target_roll(0.0), _target_pitch(0.0), _target_yaw(0.0),
     _pid_roll(0, 0, 0), _pid_pitch(0.8, 0, 0), _pid_yaw(0, 0, 0)
{
        _sub_new_position = this->create_subscription<crazyflie_msgs::msg::AttitudeCommand>(
            "/crazyflie/pid/attitude_controller",
            10,
            std::bind(&AttitudePID::_newCommandCallback, this, std::placeholders::_1));

        _sub_gps = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/crazyflie/gps",
            10,
            std::bind(&AttitudePID::_newGpsCallback, this, std::placeholders::_1));

        _sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
            "crazyflie/imu",
            10,
            std::bind(&AttitudePID::_newImuCallback, this, std::placeholders::_1));

        _pub_motor_vel = this->create_publisher<crazyflie_msgs::msg::MotorVel>(
            "/crazyflie/cmd_motor_vel",
            10);

        _cmd_motor_timer = this->create_wall_timer(
            std::chrono::milliseconds(1000 / CONTROLLER_FREQ),
            std::bind(&AttitudePID::_sendCmdMotor, this));


        _sub_tuner = this->create_subscription<crazyflie_msgs::msg::PidTuner>(
            "/crazyflie/pid_tuner",
            10,
            std::bind(&AttitudePID::_newPIDTunerCallback, this, std::placeholders::_1));

        _old_time = now();
    
}

void AttitudePID::_newPIDTunerCallback(const crazyflie_msgs::msg::PidTuner::SharedPtr pid_tune_data) {
    this->_pid_pitch.update(pid_tune_data->pitch_kp, pid_tune_data->pitch_ki, pid_tune_data->pitch_kd);
    this->_pid_roll.update(pid_tune_data->roll_kp, pid_tune_data->roll_ki, pid_tune_data->roll_kd);
    this->_pid_yaw.update(pid_tune_data->yaw_kp, pid_tune_data->yaw_ki, pid_tune_data->yaw_kd);
}

void AttitudePID::_newCommandCallback(const crazyflie_msgs::msg::AttitudeCommand::SharedPtr command) {
    _target_pitch = command->pitch;
    _target_roll = command->roll;
    _target_yaw = command->yaw;
    _input_thrust = command->thurst;
    RCLCPP_DEBUG(this->get_logger(), "target updated");
}

void AttitudePID::_newImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data) {
    tf2::Quaternion quaternion(
        imu_data->orientation.x,
        imu_data->orientation.y,
        imu_data->orientation.z,
        imu_data->orientation.w);

    tf2::Matrix3x3 mat(quaternion);
    mat.getRPY(_roll, _pitch, _yaw);

    RCLCPP_INFO(this->get_logger(), "imu data received! [r: %f, p: %f, y: %f]", _roll, _pitch, _yaw);
    RCLCPP_INFO(this->get_logger(), "p: %f", _pitch);
}

void AttitudePID::_newGpsCallback(const geometry_msgs::msg::PointStamped::SharedPtr gps_data) {
    _x = gps_data->point.x;
    _y = gps_data->point.y;
    _z = gps_data->point.z;
    RCLCPP_DEBUG(this->get_logger(), "gps data received! [x: %f, y: %f, z: %f]", _x, _y, _z);
}


void AttitudePID::_sendCmdMotor() {

    rclcpp::Duration dt = now() - _old_time; 
    
    double pid_roll     = _pid_roll.getCommand(_roll, _target_roll, dt);
    double pid_pitch    = - _pid_pitch.getCommand(_pitch, _target_pitch, dt);
    double pid_yaw      = _pid_yaw.getCommand(_yaw, _target_yaw, dt);

    auto msg = std::make_unique<crazyflie_msgs::msg::MotorVel>();
    // RCLCPP_INFO(this->get_logger(), "data received thrust %f", _pitch );
    auto mapped_thrust = ((_input_thrust - 2.5) / 7.5) / 1.5;
    msg->m1 = mapped_thrust - pid_roll - pid_pitch + pid_yaw; // GRAVITY_COMPENSATION + _input_thrust - pid_roll - pid_pitch + pid_yaw;
    msg->m2 = mapped_thrust - pid_roll + pid_pitch - pid_yaw; // GRAVITY_COMPENSATION + _input_thrust - pid_roll + pid_pitch - pid_yaw;
    msg->m3 = mapped_thrust + pid_roll + pid_pitch + pid_yaw; // GRAVITY_COMPENSATION + _input_thrust + pid_roll + pid_pitch + pid_yaw;
    msg->m4 = mapped_thrust + pid_roll - pid_pitch - pid_yaw; // GRAVITY_COMPENSATION + _input_thrust + pid_roll - pid_pitch - pid_yaw;
    _pub_motor_vel->publish(std::move(msg));

    _old_time = now();
}

int main(int argc, char** argv) 
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AttitudePID>());
    rclcpp::shutdown();
    return 0;
}
