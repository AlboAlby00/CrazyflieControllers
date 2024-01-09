#include "crazyflie_controllers/position_mpc_controller.h"
#include <iostream>
#include <Eigen/Dense>

#include "crazyflie_controllers/control_utils/ModelPredictiveController.h"

using namespace Eigen;
using namespace std;


PositionMPC::PositionMPC() :
    Node("position_mpc_controller")
{
        _sub_new_position = this->create_subscription<crazyflie_msgs::msg::PositionCommand>(
            "/crazyflie/mpc/position_controller",
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
    _mpc.computeControlInputs(_timeSteps, _f);

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
    //###############################################################################
    //#  Define the MPC algorithm parameters
    //###############################################################################

    // prediction horizon
    _f = 10;
    // control horizon
    _v = 10;

    //###############################################################################
    //# end of MPC parameter definitions
    //###############################################################################


    //###############################################################################
    //# Define the model - continuous time
    //###############################################################################


    /* Already defined some constants in the header file
    const double _Ix = 0.0000166;  // Moment of inertia around p_WB_W_x-axis, source: Julian Förster's ETH Bachelor Thesis
    const double _Iy = 0.0000167;  // Moment of inertia around p_WB_W_y-axis, source: Julian Förster's ETH Bachelor Thesis
    const double _Iz = 0.00000293;  // Moment of inertia around p_WB_W_z-axis, source: Julian Förster's ETH Bachelor Thesis
    const double _mass = 0.029;  // Mass of the quadrotor, source: Julian Förster's ETH Bachelor Thesis
    const double _g = 9.81;     // Acceleration due to gravity
    */

    Matrix <double,12,12> Ac {{0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0},
                              {0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0},
                              {0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {0, -_g, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {_g, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                              {0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0},
                              {0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0},
                              {0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0}};
    Matrix <double,12,4> Bc {{0, 0, 0, 0},
                             {0, 0, 0, 0},
                             {0, 0, 0, 0},
                             {0, 1.0/_Ix, 0, 0},
                             {0, 0, 1.0/_Iy, 0},
                             {0, 0, 0, 1.0/_Iz},
                             {0, 0, 0, 0},
                             {0, 0, 0, 0},
                             {1.0/_mass, 0, 0, 0},
                             {0, 0, 0, 0},
                             {0, 0, 0, 0},
                             {0, 0, 0, 0},};

    //Matrix <double,12,12> Cc;
    //Cc.setIdentity(); // Cc_everything


    // Cc_Rot_pos
    Matrix <double,6,12> Cc {{1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                             {0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                             {0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0},
                             {0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0},
                             {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0},
                             {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1}};




    // This is always 12-dimensional!
    //                         Rotation    angular vel   vel       position
    //                     roll,pitch,yaw, p, q, r,    u, v, w,     x, y, z
    Matrix <double,12,1> x0 {{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0},{0}};



    //  m- number of inputs
    //  r - number of outputs
    //  n - state dimension
    unsigned int n = Ac.rows();
    unsigned int m = Bc.cols();
    unsigned int r = Cc.rows();


    //###############################################################################
    //# end of model definition
    //###############################################################################

    //###############################################################################
    //# discretize the model
    //###############################################################################

    //# discretization constant

    //double sampling=0.1; // thrust is low as 2
    //double sampling=0.075; // thrust is low as 5
    double sampling=0.05; //original
    //double sampling=0.035; //
    //double sampling=0.025; //
    //double sampling=0.01; // still too much
    //double sampling=0.0001; // too much

    // # model discretization
    // identity matrix
    MatrixXd In;
    In= MatrixXd::Identity(n,n);

    MatrixXd A;
    MatrixXd B;
    MatrixXd C;
    A.resize(n,n);
    B.resize(n,m);
    C.resize(r,n);
    A=(In-sampling*Ac).inverse();
    B=A*sampling*Bc;
    C=Cc;

    //###############################################################################
    //# end of discretize the model
    //###############################################################################

    //###############################################################################
    //# form the weighting matrices
    //###############################################################################

    //# W1 matrix
    MatrixXd W1;
    W1.resize(_v*m,_v*m);
    W1.setZero();

    MatrixXd Im;
    Im= MatrixXd::Identity(m,m);

    for (unsigned int i=0; i<_v;i++)
    {
        if (i==0)
        {
            W1(seq(i*m,(i+1)*m-1),seq(i*m,(i+1)*m-1))=Im;
        }
        else
        {
            W1(seq(i*m,(i+1)*m-1),seq(i*m,(i+1)*m-1))=Im;
            W1(seq(i*m,(i+1)*m-1),seq((i-1)*m,(i)*m-1))=-Im;
        }

    }



    //# W2 matrix
    Matrix <double, Bc.cols(), Bc.cols()> Q0;
    Q0.setIdentity();

    // Calibrate W2 (i.e. Q0 and Qother) for our drone purpose

    //Q0 = Q0 * 0.00000000011; // *~0.1 -> computer thrust for z=1.0  thrust: 11.464559 (but does not fly! and does fly)
    Q0 = Q0 * 0.0000000011; //original Q0 -> computer thrust for z=1.0  thrust: 11.464559
    //Q0 = Q0 * 0.000000004; // *~4 -> computer thrust for z=1.0  thrust: 11.464559
    //Q0 = Q0 * 0.00000000875; // *~8.75 -> computer thrust for z=1.0  thrust: 11.464558 (but always this value, only accelerates)
    //Q0 = Q0 * 0.00000000875; // *~8.75 -> computer thrust for z=1.0  thrust: 11.464558 (but always this value, only accelerates)
    //Q0 = Q0 * 0.000000011; // *10 -> computer thrust for z=0.4 thrust: 4.585823 (doesn't fly)
    //Q0 = Q0 * 0.000000011; // *10 -> computer thrust for z=1.0  thrust: 11.464558 (but always this value, only accelerates)
    //Q0 = Q0 * 0.0000000175; // *~17.5 -> computer thrust for z=1.0  thrust: 11.464556
    //Q0 = Q0 * 0.000000025; // *~25 -> computer thrust for z=1.0  thrust: 11.464555
    //Q0 = Q0 * 0.00000005; // *~50 -> computer thrust for z=1.0  thrust: 11.464551
    //Q0 = Q0 * 0.00000011; // *100 -> computer thrust for z=1.0  thrust: 11.464542
    //Q0 = Q0 * 0.0000011; // *1000 -> computer thrust for z=1.0  thrust: 11.464394

    Matrix <double, Bc.cols(), Bc.cols()> Qother;
    Qother.setIdentity();
    Qother = Qother * 0.0001; //original
    //Qother = Qother * 0.01;
    //Qother = Qother * 0.000000004;


    MatrixXd W2;
    W2.resize(_v*m,_v*m);
    W2.setZero();

    for (unsigned int i=0; i<_v; i++)
    {
        if (i==0)
        {
            // this is for multivariable
            W2(seq(i*m,(i+1)*m-1),seq(i*m,(i+1)*m-1))=Q0;

            //W2(i*m,i*m)=Q0;
        }
        else
        {
            // this is for multivariable
            W2(seq(i*m,(i+1)*m-1),seq(i*m,(i+1)*m-1))=Qother;
            //W2(i*m,i*m)=Qother;

        }



    }

    MatrixXd W3;
    W3=(W1.transpose())*W2*W1;


    MatrixXd W4;
    W4.resize(_f*r,_f*r);
    W4.setZero();

    // # in the general case, this constant should be a matrix
    //double predWeight=10;

    Matrix <double, Cc.rows(), Cc.rows()> predWeight;
    predWeight.setIdentity();
    //predWeight = 10 * predWeight; //original
    predWeight = 20 * predWeight; //
    //predWeight = 100 * predWeight;

    //cout << "predWeight: " << predWeight << endl;

    for (unsigned int i=0; i < _f;i++)
    {
        //this is for multivariable
        W4(seq(i*r,(i+1)*r-1),seq(i*r,(i+1)*r-1))=predWeight;
        //W4(i*r,i*r)=predWeight;
    }


    //###############################################################################
    //# end of form the weighting matrices
    //###############################################################################

    //###############################################################################
    //# Define the reference trajectory
    //###############################################################################

    _timeSteps=14;


    //                                                Rotation    position
    //                                            roll,pitch,yaw, x, y, z
    MatrixXd desiredTrajectory_instance;
    desiredTrajectory_instance.resize(Cc.rows(),1);
    desiredTrajectory_instance << 0, 0, 0, 0, 0, 1;


    MatrixXd desiredTrajectory;
    desiredTrajectory.resize(_timeSteps * desiredTrajectory_instance.rows(), 1);


    for (unsigned int t = 0; t < _timeSteps; ++t){
        for (unsigned int r = 0; r < desiredTrajectory_instance.rows(); ++r){
            desiredTrajectory.row(t * desiredTrajectory_instance.rows() + r) = desiredTrajectory_instance.row(r);
        }

    }



    //###############################################################################
    //# end of definition of the reference trajectory
    //###############################################################################

    //###############################################################################
    //# Run the MPC algorithm
    //###############################################################################

    // create the MPC object
    _mpc = ModelPredictiveController(A, B, C,
                                     _f, _v, W3, W4, x0, desiredTrajectory);

}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionMPC>());
    rclcpp::shutdown();
    return 0;
}
