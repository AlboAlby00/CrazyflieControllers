#include "crazyflie_controllers/TinyMPC_controller.h"
#include <iostream>
#include <Eigen/Dense>
#include "tinympc/admm.hpp"

using namespace Eigen;
using namespace std;

TinyMPC::TinyMPC() :
        Node("TinyMPC_controller") {
    _sub_new_position = this->create_subscription<crazyflie_msgs::msg::PositionCommand>(
            "/crazyflie/tinyMPC/position_controller",
            10,
            std::bind(&TinyMPC::_newPositionCommandCallback, this, std::placeholders::_1));

    _sub_gps = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/crazyflie/gps",
            10,
            std::bind(&TinyMPC::_newGpsCallback, this, std::placeholders::_1));

    _sub_gps_speed = this->create_subscription<geometry_msgs::msg::Vector3>(
            "/crazyflie/gps/speed_vector",
            10,
            std::bind(&TinyMPC::_newGpsSpeedCallback, this, std::placeholders::_1));

    _sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
            "/crazyflie/imu",
            10,
            std::bind(&TinyMPC::_newImuCallback, this, std::placeholders::_1));

    _pub_motor_vel = this->create_publisher<crazyflie_msgs::msg::MotorVel>(
            "/crazyflie/cmd_motor_vel",
            10);

    _attitude_cmd_timer = this->create_wall_timer(
            std::chrono::milliseconds(50),
            std::bind(&TinyMPC::_sendCommandAttitude, this));

    _prev_time = now();

    v_WB = tf2::Vector3(0.0, 0.0, 0.0);
    omega_WB = tf2::Vector3(0.0, 0.0, 0.0);
    p_WB_W = tf2::Vector3(0.0, 0.0, 0.0);
    R_WB.setIdentity();
    p_WD_W = tf2::Vector3(0, 0, 1);

    initializeMPC();
}

// Function to convert a 4-dimensional tf2::Quaternion to 3-dimensional Rodriguez parameters
Eigen::Vector3d qtorp(const tf2::Quaternion &quaternion) {
    Eigen::Vector3d rodriguez_params;
    if (quaternion.getW() != 0) {  // Check to avoid division by zero
        rodriguez_params[0] = quaternion.getX() / quaternion.getW();
        rodriguez_params[1] = quaternion.getY() / quaternion.getW();
        rodriguez_params[2] = quaternion.getZ() / quaternion.getW();
    } else {
        std::cerr << "Error: The scalar part of the quaternion is zero." << std::endl;
    }
    return rodriguez_params;
}

void TinyMPC::_newPositionCommandCallback(const crazyflie_msgs::msg::PositionCommand::SharedPtr command) {
    /*
    if(!_is_prev_time_position_set){
        _prev_time_position = now();
        _is_prev_time_position_set = true;

    }

    // hard-coded desired position now later in the MPC method

    // Position

    // Vector (expressed in body frame B) going from frame B to a desired position D, also p_BD_B
    //p_WD_W = tf2::Vector3(command->x, command->y, command->z);

    p_BD_W = p_WD_W - p_WB_W;

    //Rotation

    R_BW = R_WB.transpose();

    // The goal here is to express the Vector p_BD_W (expressed in body frame W) going from frame B to a desired
    // position D in the Vector in the body frame B, i.e. p_BD_B.
    // Then we use this vector for the position control of the drone for correct PID gains calculation.
    // We have to update p_BD_B, because p_WD_W changes over time. This is done in the GpsCallback
    p_BD_B = R_BW * p_BD_W;

    //Velocity

    _yaw = command->yaw;
    RCLCPP_INFO(this->get_logger(),
                "target updated, p_BD_B.x(): %f, p_BD_B.y(): %f, p_BD_B.z() = %f",
                p_BD_B.x(), p_BD_B.y(), p_BD_B.z());

    double roll, pitch, yaw;
    R_WB.getRPY(roll, pitch, yaw);
     */
    // Not used right now
}

void TinyMPC::_newGpsCallback(const geometry_msgs::msg::PointStamped::SharedPtr gps_data) {
    p_WB_W = tf2::Vector3(gps_data->point.x, gps_data->point.y, gps_data->point.z);

    //Rotation & update

    p_BD_W = p_WD_W - p_WB_W;

    R_BW = R_WB.transpose();

    // The goal here is to express the Vector p_BD_W (expressed in body frame W) going from frame B to a desired
    // position D in the Vector in the body frame B, i.e. p_BD_B.
    // Then we use this vector for the position control of the drone for correct PID gains calculation.
    // We have to update p_BD_B, because p_WD_W changes over time. This is done in the GpsCallback
    p_BD_B = R_BW * p_BD_W;

    double roll, pitch, yaw;
    R_WB.getRPY(roll, pitch, yaw);
    x0
            << p_WB_W.x(), p_WB_W.y(), p_WB_W.z(), rodriguez_param_WB[0], rodriguez_param_WB[1], rodriguez_param_WB[2], v_WB.x(), v_WB.y(), v_WB.z(), omega_WB.x(), omega_WB.y(), omega_WB.z();

    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::cout << "x0 in _newGpsCallback: " << x0.transpose().format(CleanFmt) << std::endl;
}

void TinyMPC::_newGpsSpeedCallback(const geometry_msgs::msg::Vector3 gps_speedVec) {
    v_WB = tf2::Vector3(gps_speedVec.x, gps_speedVec.y, gps_speedVec.z);
}

void TinyMPC::_newImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data) {
    quaternion_WB = tf2::Quaternion(
            imu_data->orientation.x,
            imu_data->orientation.y,
            imu_data->orientation.z,
            imu_data->orientation.w);

    // Rotation going from world frame W to the body frame B, cannot name it _R_WB
    R_WB = tf2::Matrix3x3(quaternion_WB);


    double roll, pitch, yaw;
    R_WB.getRPY(roll, pitch, yaw);

    //Frame B's angular velocity in frame W
    omega_WB = tf2::Vector3(imu_data->angular_velocity.x, imu_data->angular_velocity.y, imu_data->angular_velocity.z);


    //RCLCPP_INFO(this->get_logger(),
    //             "imu data received! [R_WB_roll: %f, R_WB_pitch: %f, R_WB_yaw: %f]",
    //             roll, pitch, yaw);

    rodriguez_param_WB = qtorp(quaternion_WB);
    x0
            << p_WB_W.x(), p_WB_W.y(), p_WB_W.z(), rodriguez_param_WB[0], rodriguez_param_WB[1], rodriguez_param_WB[2], v_WB.x(), v_WB.y(), v_WB.z(), omega_WB.x(), omega_WB.y(), omega_WB.z();

    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::cout << "x0 in _newImuCallback: " << x0.transpose().format(CleanFmt) << std::endl;
}

void TinyMPC::_sendCommandAttitude() {
    auto start = std::chrono::high_resolution_clock::now();

    TinySolver solver{&settings, &cache, &work};

    // Make sure in glob_opts.hpp:
    // - NSTATES = 12, NINPUTS=4
    // - NHORIZON = anything you want
    // - tinytype = float if you want to run on microcontrollers
    // States: x (m), y, z, phi, theta, psi, dx, dy, dz, dphi, dtheta, dpsi
    // phi, theta, psi are NOT Euler angles, they are Rodiguez parameters
    // check this paper for more details: https://ieeexplore.ieee.org/document/9326337
    // Inputs: u1, u2, u3, u4 (motor thrust 0-1, order from Crazyflie)

    // Hovering setpoint
    tiny_VectorNx Xref_origin;
    Xref_origin << 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0; // supposed to be p_WD_W
    work.Xref = Xref_origin.replicate<1, NHORIZON>();

    //Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    //std::cout << "work.Xref: " << work.Xref.transpose().format(CleanFmt) << std::endl;

    // Update measurement
    work.x.col(0) = x0;

    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::cout << "work.x.col(0): " << work.x.col(0).transpose().format(CleanFmt) << std::endl;

    //printf("tracking error: %.4f\n", (x0 - work.Xref.col(1)).norm());
    auto error = work.Xref.col(0) - work.x.col(0);
    std::cout << "error in _sendCommandAttitude: " << error.transpose().format(CleanFmt) << std::endl;


    int exitflag = tiny_solve(&solver);

    if (exitflag == 0) {
        printf("HOORAY! Solved with no error!\n");
    } else {
        printf("OOPS! Something went wrong!\n");
        std::cout << "rodriguez_param_WB: " << rodriguez_param_WB.transpose().format(CleanFmt) << std::endl;
        std::cout << "Quaternion x: " << quaternion_WB.getX()
                  << " y: " << quaternion_WB.getY()
                  << " z: " << quaternion_WB.getZ()
                  << " w: " << quaternion_WB.getW() << std::endl;
        //std::cout << "2ndtime: work.x.col(0) in _sendCommandAttitude: " << work.x.col(0).transpose().format(CleanFmt) << std::endl;
        //std::cout << "2ndtime: error in _sendCommandAttitude: " << error.transpose().format(CleanFmt) << std::endl;
    }


    auto msg = std::make_unique<crazyflie_msgs::msg::MotorVel>();

    auto u = work.u.col(0);

    msg->m1 = GRAVITY_COMPENSATION + u(0);
    msg->m2 = GRAVITY_COMPENSATION + u(1);
    msg->m3 = GRAVITY_COMPENSATION + u(2);
    msg->m4 = GRAVITY_COMPENSATION + u(3);

    printf("u1: %.4f, u2: %.4f, u3: %.4f, u4: %.4f\n", u(0), u(1), u(2), u(3));

    auto u_prev = u;

    if ((u(0) * u_prev(0) < 0.0) || (u(1) * u_prev(1) < 0.0) || (u(2) * u_prev(2) < 0.0) || (u(3) * u_prev(3) < 0.0)) {
        printf("At least one sign of the the vector u changed!\n");
    }

    _pub_motor_vel->publish(std::move(msg));

    auto stop = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);

    std::cout << "Time taken by function: " << duration.count() << " milliseconds" << std::endl;
}

void TinyMPC::initializeMPC() {
    cache.rho = rho_value;
    cache.Kinf = Eigen::Map<Matrix<tinytype, NINPUTS, NSTATES, Eigen::RowMajor>>(Kinf_data);
    cache.Pinf = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Pinf_data);
    cache.Quu_inv = Eigen::Map<Matrix<tinytype, NINPUTS, NINPUTS, Eigen::RowMajor>>(Quu_inv_data);
    cache.AmBKt = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(AmBKt_data);
    cache.coeff_d2p = Eigen::Map<Matrix<tinytype, NSTATES, NINPUTS, Eigen::RowMajor>>(coeff_d2p_data);

    work.Adyn = Eigen::Map<Matrix<tinytype, NSTATES, NSTATES, Eigen::RowMajor>>(Adyn_data);
    work.Bdyn = Eigen::Map<Matrix<tinytype, NSTATES, NINPUTS, Eigen::RowMajor>>(Bdyn_data);
    work.Q = Eigen::Map<tiny_VectorNx>(Q_data);
    work.R = Eigen::Map<tiny_VectorNu>(R_data);
    work.u_min = tiny_MatrixNuNhm1::Constant(-10);
    work.u_max = tiny_MatrixNuNhm1::Constant(10);
    work.x_min = tiny_MatrixNxNh::Constant(-5);
    work.x_max = tiny_MatrixNxNh::Constant(5);

    work.Xref = tiny_MatrixNxNh::Zero();
    work.Uref = tiny_MatrixNuNhm1::Zero();

    work.x = tiny_MatrixNxNh::Zero();
    work.q = tiny_MatrixNxNh::Zero();
    work.p = tiny_MatrixNxNh::Zero();
    work.v = tiny_MatrixNxNh::Zero();
    work.vnew = tiny_MatrixNxNh::Zero();
    work.g = tiny_MatrixNxNh::Zero();

    work.u = tiny_MatrixNuNhm1::Zero();
    work.r = tiny_MatrixNuNhm1::Zero();
    work.d = tiny_MatrixNuNhm1::Zero();
    work.z = tiny_MatrixNuNhm1::Zero();
    work.znew = tiny_MatrixNuNhm1::Zero();
    work.y = tiny_MatrixNuNhm1::Zero();

    work.primal_residual_state = 0;
    work.primal_residual_input = 0;
    work.dual_residual_state = 0;
    work.dual_residual_input = 0;
    work.status = 0;
    work.iter = 0;

    settings.abs_pri_tol = 0.001;
    settings.abs_dua_tol = 0.001;
    settings.max_iter = 100;
    settings.check_termination = 1;
    settings.en_input_bound = 10;
    settings.en_state_bound = 10;
}


int main(int argc, char const *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TinyMPC>());
    rclcpp::shutdown();
    return 0;
}
