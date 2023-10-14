#include "crazyflie_controllers/attitude_pid_controller.h"

AttitudePID::AttitudePID()
    : Node("attitude_pid_controller"), _target_z(5.0), _target_roll(0.0), _target_pitch(0.0), _target_yaw(0.0),
            _pid_z(10, 50, 5, CONTROLLER_FREQ), _pid_roll(2, 0, 10, CONTROLLER_FREQ), _pid_pitch(2, 0, 30, CONTROLLER_FREQ)
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
        "/crazyflie/imu",
        10,
        std::bind(&AttitudePID::_newImuCallback, this, std::placeholders::_1));

    _sub_pid_tuner = this->create_subscription<crazyflie_msgs::msg::PidTuner>(
        "/crazyflie/pid_tuner",
        10,
        std::bind(&AttitudePID::_pidTuneCallback, this, std::placeholders::_1));


    _pub_motor_vel = this->create_publisher<crazyflie_msgs::msg::MotorVel>(
        "/crazyflie/cmd_motor_vel",
        10);

    _cmd_motor_timer = this->create_wall_timer(
        std::chrono::milliseconds(1000 / CONTROLLER_FREQ),
        std::bind(&AttitudePID::_sendCmdMotor, this));

    _old_time = now();
  
}

void AttitudePID::_newCommandCallback(const crazyflie_msgs::msg::AttitudeCommand::SharedPtr command) {
    _target_pitch = command->pitch;
    _target_roll = command->roll;
    _target_yaw = command->yaw;
    _target_z = command->thurst;
    RCLCPP_INFO(this->get_logger(), "target updated");

}

void AttitudePID::_newImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data) {
    tf2::Quaternion quaternion(
        imu_data->orientation.x,
        imu_data->orientation.y,
        imu_data->orientation.z,
        imu_data->orientation.w);

    tf2::Matrix3x3 mat(quaternion);
    mat.getRPY(_roll, _pitch, _yaw);
    RCLCPP_DEBUG(this->get_logger(), "imu data received! [r: %f, p: %f, y: %f]", _roll, _pitch, _yaw);
}

void AttitudePID::_newGpsCallback(const geometry_msgs::msg::PointStamped::SharedPtr gps_data) {
    _x = gps_data->point.x;
    _y = gps_data->point.y;
    _z = gps_data->point.z;
    RCLCPP_DEBUG(this->get_logger(), "gps data received! [x: %f, y: %f, z: %f]", _x, _y, _z);
}

void AttitudePID::_pidTuneCallback(const crazyflie_msgs::msg::PidTuner::SharedPtr tune_msg)
{
    _pid_z.update(tune_msg->thurst_kp, tune_msg->thurst_ki, tune_msg->thurst_kd);
}

void AttitudePID::_sendCmdMotor() {

    rclcpp::Duration dt = now() - _old_time; 
    double throttle = _pid_z.getCommand(_z, _target_z, dt);
    //double roll = _pid_roll.getCommand(_roll, _target_roll, dt);
    //double pitch = _pid_pitch.getCommand(_pitch, _target_pitch, dt);
    double roll = 0;
    double pitch = 0;

    auto msg = std::make_unique<crazyflie_msgs::msg::MotorVel>();
    msg->m1 = GRAVITY_COMPENSATION + throttle - roll + pitch;
    msg->m2 = GRAVITY_COMPENSATION + throttle - roll - pitch;
    msg->m3 = GRAVITY_COMPENSATION + throttle + roll - pitch;
    msg->m4 = GRAVITY_COMPENSATION + throttle + roll + pitch;
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
