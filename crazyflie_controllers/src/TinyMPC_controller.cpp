#include "crazyflie_controllers/TinyMPC_controller.h"
#include <iostream>
#include <Eigen/Dense>

#include "crazyflie_controllers/control_utils/ModelPredictiveController.h"
#include "tinympc/admm.hpp"

using namespace Eigen;
using namespace std;


PositionMPC::PositionMPC() :
    Node("TinyMPC_controller")
{
        _sub_new_position = this->create_subscription<crazyflie_msgs::msg::PositionCommand>(
            "/crazyflie/tinyMPC/position_controller",
            10,
            std::bind(&PositionMPC::_newPositionCommandCallback, this, std::placeholders::_1));

        _sub_gps = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/crazyflie/gps",
            10,
            std::bind(&PositionMPC::_newGpsCallback, this, std::placeholders::_1));

        _sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
            "/crazyflie/imu",
            10,
            std::bind(&PositionMPC::_newImuCallback, this, std::placeholders::_1));

        _pub_attutude_cmd = this->create_publisher<crazyflie_msgs::msg::AttitudeCommand>(
            "/crazyflie/pid/attitude_controller",
            10);

        _attitude_cmd_timer = this->create_wall_timer(
                std::chrono::milliseconds(30),
            std::bind(&PositionMPC::_sendCommandAttitude, this));

        //std::chrono::milliseconds(1000 / CONTROLLER_FREQ)
        // cout << "1000 / CONTROLLER_FREQ: " << 1000 / CONTROLLER_FREQ << endl;

        _prev_time = now();
        _is_prev_time_position_set = false;

        v_WB = tf2::Vector3(0.0, 0.0, 0.0);
        omega_WB = tf2::Vector3(0.0, 0.0, 0.0);
        p_WB_W = tf2::Vector3(0.0, 0.0, 0.0);
        R_WB.setIdentity();
        _desiredControlSetByCallback = false;

        InitializeMPC();
}

void PositionMPC::_newPositionCommandCallback(const crazyflie_msgs::msg::PositionCommand::SharedPtr command)
{
    if(!_is_prev_time_position_set){
        _prev_time_position = now();
        _is_prev_time_position_set = true;

    }

    // hard-coded desired position now later in the MPC method

    // Position

    // Vector (expressed in body frame B) going from frame B to a desired position D, also p_BD_B
    p_WD_W = tf2::Vector3(command->x, command->y, command->z);

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

    MatrixXd desiredControlTrajectoryTotalInput;

    MatrixXd desiredTrajectory_instance;
    //                            Cc.rows() = 6 for Rotation (3D) and Position (3D) observation
    desiredTrajectory_instance.resize(6 ,1);
    //              assuming that the world Coordinate system is perfectly leveled
    //                                       Rotation,                        Position
    //                            R_WD_roll, R_WD_pitch, R_WD_yaw, p_WD_W_x,   p_WD_W_y,   p_WD_W_z
    // desiredTrajectory_instance << 0,         0,          0,        command->x, command->y, command->z; //WRONG!
    desiredTrajectory_instance << 0,         0,          0,        p_BD_B.x(), p_BD_B.y(), p_BD_B.z();

    cout << "p_BD_B.z(): " << p_BD_B.z() << endl;


    MatrixXd desiredTrajectory;
    desiredTrajectory.resize(_timeSteps * desiredTrajectory_instance.rows(), 1);


    for (unsigned int t = 0; t < _timeSteps; ++t){
        for (unsigned int r = 0; r < desiredTrajectory_instance.rows(); ++r){
            desiredTrajectory.row(t * desiredTrajectory_instance.rows() + r) = desiredTrajectory_instance.row(r);
        }

    }

    _mpc.setDesiredControlTrajectoryTotal(desiredTrajectory);

}

void PositionMPC::_newGpsCallback(const geometry_msgs::msg::PointStamped::SharedPtr gps_data)
{
    if(!_is_prev_time_position_set){
        _prev_time_position = now();
        _is_prev_time_position_set = true;

        //This will be set to compare to the next position to generate the velocity
        p_WB_W_prev = tf2::Vector3(gps_data->point.x, gps_data->point.y, gps_data->point.z);

        cout << "gps_data->point.z in _newGpsCallback: \n" << gps_data->point.z << endl;

        //First velocity vector will be zero, because there are no two positions to generate velocity from
        v_WB = tf2::Vector3(0.0, 0.0, 0.0);

    } else {

        rclcpp::Duration dt = now() - _prev_time_position; // assuming that a previous time is set

        _prev_time_position = now(); // setting the previous time of the position to now for next iteration to compare

        // Position p_WD_W

        // Vector (expressed in body frame B) going from frame B to a desired position D, also p_BD_B
        p_WB_W = tf2::Vector3(gps_data->point.x, gps_data->point.y, gps_data->point.z);

        //Velocity
        double dt_seconds = dt.seconds();

        if (dt_seconds > 0) {
            // Point B's (The body frame B of the drone) translational velocity in frame W
            v_WB = (p_WB_W - p_WB_W_prev) / dt_seconds;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Duration is non-positive, cannot compute velocity.");
            // set v_WB to a default value
            v_WB = tf2::Vector3(0.0, 0.0, 0.0); // Setting velocity to zero
        }

    } // End else

    //Rotation & update

    p_BD_W = p_WD_W - p_WB_W;

    R_BW = R_WB.transpose();

    // The goal here is to express the Vector p_BD_W (expressed in body frame W) going from frame B to a desired
    // position D in the Vector in the body frame B, i.e. p_BD_B.
    // Then we use this vector for the position control of the drone for correct PID gains calculation.
    // We have to update p_BD_B, because p_WD_W changes over time. This is done in the GpsCallback
    p_BD_B = R_BW * p_BD_W;

    //RCLCPP_INFO(this->get_logger(),
    //             "GPS updated, p_WB.x: %f, p_WB.y(): %f, p_WB.z = %f", p_WB_W.x(), p_WB_W.y(), p_WB_W.z());

    MatrixXd x0(12, 1);
    double roll, pitch, yaw;
    R_WB.getRPY(roll, pitch, yaw);
    x0 << roll, pitch, yaw, omega_WB.x(), omega_WB.y(), omega_WB.z(), v_WB.x(), v_WB.y(), v_WB.z(), 0, 0, 0;

    cout << "v_WB.z() of x0 in _newGpsCallback: \n" << x0(8,0) << endl;
    //std::lock_guard<std::mutex> guard(_mpc_mutex);
    _mpc.setx0(x0);

    // Position feedback from world frame should be communicated to the MPC controller in the update of the desired
    // trajectory


    MatrixXd desiredControlTrajectoryTotalInput;

    MatrixXd desiredTrajectory_instance;
    //                            Cc.rows() = 6 for Rotation (3D) and Position (3D) observation
    desiredTrajectory_instance.resize(6 ,1);
    //              assuming that the world Coordinate system is perfectly leveled
    //                                       Rotation,                        Position
    //                            R_WD_roll, R_WD_pitch, R_WD_yaw, p_WD_W_x,   p_WD_W_y,   p_WD_W_z
    // desiredTrajectory_instance << 0,         0,          0,        command->x, command->y, command->z; //WRONG!
    desiredTrajectory_instance << 0,         0,          0,        p_BD_B.x(), p_BD_B.y(), p_BD_B.z();

    cout << "p_BD_B.z() in _newGpsCallback: " << p_BD_B.z() << endl;


    MatrixXd desiredTrajectory;
    desiredTrajectory.resize(_timeSteps * desiredTrajectory_instance.rows(), 1);


    for (unsigned int t = 0; t < _timeSteps; ++t){
        for (unsigned int r = 0; r < desiredTrajectory_instance.rows(); ++r){
            desiredTrajectory.row(t * desiredTrajectory_instance.rows() + r) = desiredTrajectory_instance.row(r);
        }

    }

    _mpc.setDesiredControlTrajectoryTotal(desiredTrajectory);
}

void PositionMPC::_newImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data) {
    tf2::Quaternion quaternion(
            imu_data->orientation.x,
            imu_data->orientation.y,
            imu_data->orientation.z,
            imu_data->orientation.w);

    // Rotation going from world frame W to the body frame B, cannot name it _R_WB
    R_WB = tf2::Matrix3x3(quaternion);


    double roll, pitch, yaw;
    R_WB.getRPY(roll, pitch, yaw);

    //Frame B's angular velocity in frame W
    omega_WB = tf2::Vector3(imu_data->angular_velocity.x, imu_data->angular_velocity.y, imu_data->angular_velocity.z);


    //RCLCPP_INFO(this->get_logger(),
    //             "imu data received! [R_WB_roll: %f, R_WB_pitch: %f, R_WB_yaw: %f]",
    //             roll, pitch, yaw);

    MatrixXd x0(12, 1);
    x0 << roll, pitch, yaw, omega_WB.x(), omega_WB.y(), omega_WB.z(), v_WB.x(), v_WB.y(),  v_WB.z(), 0, 0, 0;

    cout << "v_WB.z() of x0 in _newImuCallback: \n" << x0(8,0) << endl;

    //std::lock_guard<std::mutex> guard(_mpc_mutex);
    _mpc.setx0(x0);

}

void PositionMPC::_sendCommandAttitude()
{
    // Quadrotor hovering example

    // This script is just to show how to use the library,
    // the data for this example is not tuned for our Crazyflie demo. Check the firmware code for more details.

    // Make sure in glob_opts.hpp:
    // - NSTATES = 12, NINPUTS=4
    // - NHORIZON = anything you want
    // - tinytype = float if you want to run on microcontrollers
    // States: x (m), y, z, phi, theta, psi, dx, dy, dz, dphi, dtheta, dpsi
    // phi, theta, psi are NOT Euler angles, they are Rodiguez parameters
    // check this paper for more details: https://ieeexplore.ieee.org/document/9326337
    // Inputs: u1, u2, u3, u4 (motor thrust 0-1, order from Crazyflie)
    printf("tracking error: %.4f\n", (x0 - work.Xref.col(1)).norm());

    // 1. Update measurement
    //work.x.col(0) = x0;

    // 2. Update reference (if needed)

    // 3. Reset dual variables (if needed)
    // work.y = tiny_MatrixNuNhm1::Zero();
    // work.g = tiny_MatrixNxNh::Zero();

    // 4. Solve MPC problem
    tiny_solve(&solver);

    // std::cout << work.iter << std::endl;
    // std::cout << work.u.col(0).transpose().format(CleanFmt) << std::endl;

    // 5. Simulate forward
    x1 = work.Adyn * x0 + work.Bdyn * work.u.col(0);
    x0 = x1;

    // std::cout << x0.transpose().format(CleanFmt) << std::endl;

    auto msg = std::make_unique<crazyflie_msgs::msg::AttitudeCommand>();

    float thrust = static_cast<float>(_mpc.inputs(0,0)); // Access the optimized thrust value
    float tau_x = static_cast<float>(_mpc.inputs(1,0)); // Access the optimized torque around x-axis
    float tau_y = static_cast<float>(_mpc.inputs(2,0)); // Access the optimized torque around y-axis
    float tau_z =  static_cast<float>(_mpc.inputs(3,0)); // Access the optimized torque around z-axis

    RCLCPP_INFO(this->get_logger(),
                "MPC controls computed! [thrust: %f, tau_x: %f, tau_y: %f, tau_z: %f]",
                thrust, tau_x, tau_y, tau_z);


    msg->pitch = tau_y;
    msg->roll = tau_x;
    msg->thurst = thrust;
    msg->yaw = tau_z;

    _pub_attutude_cmd->publish(std::move(msg));

    _prev_time = now();

}

void PositionMPC::InitializeMPC() {
    TinyCache cache;
    TinyWorkspace work;
    TinySettings settings;
    TinySolver solver{&settings, &cache, &work};

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
    work.u_min = tiny_MatrixNuNhm1::Constant(-0.5);
    work.u_max = tiny_MatrixNuNhm1::Constant(0.5);
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



    // Hovering setpoint
    tiny_VectorNx Xref_origin;
    Xref_origin << 0, 0, 2, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    work.Xref = Xref_origin.replicate<1, NHORIZON>();

    // Initial state
    x0 << 0, 1, 0, 0.2, 0, 0, 0.1, 0, 0, 0, 0, 0;
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionMPC>());
    rclcpp::shutdown();
    return 0;
}