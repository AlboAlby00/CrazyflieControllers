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

    _cmd_timer = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&TinyMPC::_sendCommand, this));

    v_WB = tf2::Vector3(0.0, 0.0, 0.0);
    omega_WB = tf2::Vector3(0.0, 0.0, 0.0);
    p_WB_W = tf2::Vector3(0.0, 0.0, 0.0);
    R_WB.setIdentity();
    p_WD_W = tf2::Vector3(0, 0, 1);

    initializeMPC();
}

void TinyMPC::_newPositionCommandCallback(const crazyflie_msgs::msg::PositionCommand::SharedPtr command) {
    std::lock_guard<std::mutex> guard(_mutex);

    p_WD_W = tf2::Vector3(command->x, command->y, command->z); // to avoid stderr about unused parameter

}

void TinyMPC::_newGpsCallback(const geometry_msgs::msg::PointStamped::SharedPtr gps_data) {
    std::lock_guard<std::mutex> guard(_mutex);
    p_WB_W = tf2::Vector3(gps_data->point.x, gps_data->point.y, gps_data->point.z);

    //Rotation & update

    p_BD_W = p_WD_W - p_WB_W;

    R_BW = R_WB.transpose();

    // The goal here is to express the Vector p_BD_W (expressed in body frame W) going from frame B to a desired
    // position D in the Vector in the body frame B, i.e. p_BD_B.
    // Then we use this vector for the TinyMPC control of the drone.
    // We have to update p_BD_B, because p_WD_W changes over time. This is done in the GpsCallback
    p_BD_B = R_BW * p_BD_W;

    x0
            << p_WB_W.x(), p_WB_W.y(), p_WB_W.z(), rodriguez_param_WB[0], rodriguez_param_WB[1], rodriguez_param_WB[2], v_WB.x(), v_WB.y(), v_WB.z(), omega_WB.x(), omega_WB.y(), omega_WB.z();

    //Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    //std::cout << "x_ref in _newGpsCallback: " << x_ref.transpose().format(CleanFmt) << std::endl;
}

void TinyMPC::_newGpsSpeedCallback(const geometry_msgs::msg::Vector3 gps_speedVec) {
    std::lock_guard<std::mutex> guard(_mutex);
    v_WB = tf2::Vector3(gps_speedVec.x, gps_speedVec.y, gps_speedVec.z);
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

void TinyMPC::_newImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data) {
    std::lock_guard<std::mutex> guard(_mutex);
    quaternion_WB = tf2::Quaternion(
            imu_data->orientation.x,
            imu_data->orientation.y,
            imu_data->orientation.z,
            imu_data->orientation.w);

    // Rotation going from world frame W to the body frame B, cannot name it _R_WB
    R_WB = tf2::Matrix3x3(quaternion_WB);


    //Frame B's angular velocity in frame W
    omega_WB = tf2::Vector3(imu_data->angular_velocity.x, imu_data->angular_velocity.y, imu_data->angular_velocity.z);


    rodriguez_param_WB = qtorp(quaternion_WB);
    x0
            << p_WB_W.x(), p_WB_W.y(), p_WB_W.z(), rodriguez_param_WB[0], rodriguez_param_WB[1], rodriguez_param_WB[2], v_WB.x(), v_WB.y(), v_WB.z(), omega_WB.x(), omega_WB.y(), omega_WB.z();

    //Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    //std::cout << "x_ref in _newImuCallback: " << x_ref.transpose().format(CleanFmt) << std::endl;
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
    settings.en_input_bound = 1;
    settings.en_state_bound = 1;
}

void TinyMPC::_sendCommand() {
    auto start = std::chrono::high_resolution_clock::now();
    std::lock_guard<std::mutex> guard(_mutex);

    TinySolver solver{&settings, &cache, &work};

    // Make sure in glob_opts.hpp:
    // - NSTATES = 12, NINPUTS=4
    // - NHORIZON = anything you want
    // - tinytype = float if you want to run on microcontrollers
    // States: x (m), y, z, phi, theta, psi, dx, dy, dz, dphi, dtheta, dpsi
    // phi, theta, psi are NOT Euler angles, they are Rodiguez parameters
    // check this paper for more details: https://ieeexplore.ieee.org/document/9326337
    // Inputs: u1, u2, u3, u4 (motor thrust 0-1, order from Crazyflie)

    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");


    // Alternative: Hovering setpoint static
    tiny_VectorNx x_ref_origin_static;
    //x_ref_origin_static << 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    //x_ref_origin_static << 1.477e-18, 4.548e-18, 0.985,  4.086e-18,  2.676e-18, -7.248e-05,  1.372e-18,  2.708e-19,  1.409e-15,  6.142e-18,  1.443e-16, -3.038e-18;
    //x_ref_origin_static << 3.882e-18, -1.094e-18,  0.985,  2.301e-18,  4.219e-19, -7.248e-05,  4.394e-19, -1.368e-19, 0,  5.575e-17,  7.286e-17, -6.772e-19;
    //x_ref_origin_static << p_WD_W.x(), p_WD_W.y(), p_WD_W.z(), 0, 0, 0, 0, 0, 0, 0, 0, 0;

    //work.Xref = x_ref_origin_static.replicate<1, NHORIZON>();

    // Hovering setpoint dynamic
    // Position of x_ref is p_WD_W
    x_ref_origin_static << p_WD_W.x(), p_WD_W.y(), p_WD_W.z(), 0, 0, 0, 0, 0, 0, 0, 0, 0;
    std::cout << "x_ref before the solver: " << x_ref.transpose().format(CleanFmt) << std::endl;
    //tiny_VectorNx x_ref_origin_static_copy =  x_ref;

    work.Xref = x_ref_origin_static.replicate<1, NHORIZON>();

    /*
    //bool areEqual = x_ref_origin_static.isApprox(x_ref, 1e-2);
    //std::cout << "Are static and dynamic x_ref approximately equal? " << areEqual << std::endl;


    if(!areEqual){
       cout << "NOT EQUAL!" << endl;
       auto difference =  x_ref_origin_static - x_ref;
       std::cout << "difference: " << difference.transpose().format(CleanFmt) << std::endl;
    }
    */

    std::cout << "work.Xref.col(0) in _sendCommand: " << work.Xref.col(0).transpose().format(CleanFmt) << std::endl;

    // Update measurement
    //x0 << 0, 0, 0.01, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    work.x.col(0) = x0; // position is p_BD_B (MIT Tedrake notation)


    std::cout << "work.x.col(0): " << work.x.col(0).transpose().format(CleanFmt) << std::endl;

    //printf("tracking error: %.4f\n", (x0 - work.Xref.col(1)).norm());
    auto error = work.Xref.col(0) - work.x.col(0);
    std::cout << "Tracking error in _sendCommand: " << error.transpose().format(CleanFmt) << std::endl;

    auto x_ref_before = x_ref;

    int exitflag = tiny_solve(&solver);

    std::cout << "x_ref after the solver: " << x_ref.transpose().format(CleanFmt) << std::endl;
    if(!x_ref.isApprox(x_ref_before)){
        cout << "x_ref NOT EQUAL x_ref_before!" << endl;
        auto difference =  x_ref_before - x_ref;
        std::cout << "difference: " << difference.transpose().format(CleanFmt) << std::endl;
    }

    if (exitflag == 0) {
        printf("HOORAY! Solved with no error!\n");
    } else {
        printf("OOPS! Something went wrong!\n");
        std::cout << "rodriguez_param_WB: " << rodriguez_param_WB.transpose().format(CleanFmt) << std::endl;
        std::cout << "Quaternion x: " << quaternion_WB.getX()
                  << " y: " << quaternion_WB.getY()
                  << " z: " << quaternion_WB.getZ()
                  << " w: " << quaternion_WB.getW() << std::endl;
        //std::cout << "2ndtime: work.x.col(0) in _sendCommand: " << work.x.col(0).transpose().format(CleanFmt) << std::endl;
        //std::cout << "2ndtime: error in _sendCommand: " << error.transpose().format(CleanFmt) << std::endl;
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


int main(int argc, char const *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TinyMPC>());
    rclcpp::shutdown();
    return 0;
}