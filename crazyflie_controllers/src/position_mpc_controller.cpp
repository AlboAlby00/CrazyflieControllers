#include "crazyflie_controllers/position_mpc_controller.h"

PositionMPC::PositionMPC() :
    Node("position_mpc_controller"), _input_yaw(0.0), _target_x(0.0), _target_y(0.0), _target_z(0.0)
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
            std::chrono::milliseconds(1000 / CONTROLLER_FREQ),
            std::bind(&PositionMPC::_sendCommandAttitude, this));

        _old_time = now();
  
}

void PositionMPC::_newPositionCommandCallback(const crazyflie_msgs::msg::PositionCommand::SharedPtr command)
{
    _target_x = command->x;
    _target_y = command->y;
    _target_z = command->z;
    _yaw = command->yaw;
    RCLCPP_INFO(this->get_logger(), "target updated, x: %f, y: %f, z= %f", _target_x, _target_y, _target_z);
}

void PositionMPC::_newGpsCallback(const geometry_msgs::msg::PointStamped::SharedPtr gps_data)
{
    _x = gps_data->point.x;
    _y = gps_data->point.y;
    _z = gps_data->point.z;

}

void PositionMPC::_newImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_data) {
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

void PositionMPC::_sendCommandAttitude()
{
    rclcpp::Duration dt = now() - _old_time;

    // Constants
    const double Ix = 0.0142;  // Moment of inertia around p_WB_W_x-axis
    const double Iy = 0.0142;  // Moment of inertia around p_WB_W_y-axis
    const double Iz = 0.0284;  // Moment of inertia around p_WB_W_z-axis
    const double m = 1.56779;  // Mass of the quadrotor
    const double g = 9.81;     // Acceleration due to gravity
    const double samplingTime = 0.01; // Sampling time for real-time control

    // Initialize ACADO environment
    DifferentialState p_WB_W_x, p_WB_W_y, p_WB_W_z;         // Position
    DifferentialState R_WB_phi, R_WB_theta, R_WB_psi; // Orientation (roll, pitch, yaw)
    DifferentialState u, v, w;         // Body frame velocities
    DifferentialState p, q, r;         // Angular rates

    Control ft;                        // Thrust
    Control tau_x, tau_y, tau_z;       // Torques

    // Define the linearized dynamics of the quadrotor
    DifferentialEquation f;

    /*
    // Angular rates dynamics
    f << dot(R_WB_phi) == p + r*R_WB_theta;
    f << dot(R_WB_theta) == q - r*R_WB_phi;
    f << dot(R_WB_psi) == r + q*R_WB_phi;

    // Angular accelerations dynamics
    f << dot(p) == (tau_x - (Iz - Iy)*r*q) / Ix;
    f << dot(q) == (tau_y - (Ix - Iz)*r*p) / Iy;
    f << dot(r) == (tau_z - (Iy - Ix)*p*q) / Iz;

    // Linear accelerations dynamics
    f << dot(u) == r*v - q*w - g*R_WB_theta + ft/m;
    f << dot(v) == p*w - r*u + g*R_WB_phi;
    f << dot(w) == q*u - p*v + g - ft/m;

    // Linear velocities dynamics
    f << dot(p_WB_W_x) == u;
    f << dot(p_WB_W_y) == v;
    f << dot(p_WB_W_z) == w;
     */

    // Angular rates dynamics
    f << dot(R_WB_phi) == p;
    f << dot(R_WB_theta) == q;
    f << dot(R_WB_psi) == r;

    // Angular accelerations dynamics
    f << dot(p) == tau_x / Ix;
    f << dot(q) == tau_y / Iy;
    f << dot(r) == tau_z / Iz;

    // Linear accelerations dynamics
    f << dot(u) == -g * R_WB_theta;
    f << dot(v) == g * R_WB_phi;
    f << dot(w) == ft/m;

    // Linear velocities dynamics
    f << dot(p_WB_W_x) == u;
    f << dot(p_WB_W_y) == v;
    f << dot(p_WB_W_z) == w;

    // Set up the MPC optimization problem
    // Prediction horizon of 2 seconds, with sampling time intervals
    OCP ocp(0.0, 2.0, (int)(2.0/samplingTime));

    Function h;
    h << p_WB_W_x << p_WB_W_y << p_WB_W_z << R_WB_phi << R_WB_theta << R_WB_psi << u << v << w << p << q << r;
    h << ft << tau_x << tau_y << tau_z;

    //DMatrix Q(h.getDim(), h.getDim());
    //Q.eye(); // Simple identity matrix for weighting
    //const int dim = h.getDim();
    //uint dim = static_cast<uint>(h.getDim());
    //DMatrix Q(dim, dim);
    //DMatrix Q(dim, dim);
    //Q.setIdentity();
    /* uint dim = static_cast<uint>(h.getDim());
    DMatrix Q(dim, dim); // Initializes a dim p_WB_W_x dim matrix with zeros

    // Manually set the diagonal elements to 1 to create an identity matrix
    for (uint i = 0; i < dim; ++i) {
        Q(i, i) = 1.0;
    } */

    /*std::vector<std::vector<double>> matrixData = {
            {1.0, 2.0},
            {3.0, 4.0}
    };

    unsigned numRows = 2;
    unsigned numCols = 2;
    DMatrix Q(numRows, numCols, matrixData);*/

    //std::vector<std::vector<double>> matrixData = {
    //        {1.0, 2.0},
    //        {3.0, 4.0}
    //};

    //unsigned dim = 2;


    //DMatrix Q(dim, dim, matrixData);

    unsigned int dim = static_cast<unsigned int>(h.getDim());
    std::vector<std::vector<double>> identityData(dim, std::vector<double>(dim, 0.0));
    for (unsigned int i = 0; i < dim; ++i) {
        identityData[i][i] = 1.0;
    }
    DMatrix Q(dim, dim, identityData);


    ocp.minimizeLSQ(Q, h);

    // Subject to the differential equation
    ocp.subjectTo(f);

    // Add constraints for states and control inputs
    // Example constraints:
    ocp.subjectTo(-10.0 <= p_WB_W_x <= 10.0); // p_WB_W_x position constraint
    ocp.subjectTo(-10.0 <= p_WB_W_y <= 10.0); // p_WB_W_y position constraint
    ocp.subjectTo(-10.0 <= p_WB_W_z <= 10.0); // p_WB_W_z position constraint
    // ... Add other constraints as necessary

    // Configure the solver for real-time optimization
    RealTimeAlgorithm solver(ocp, samplingTime);
    solver.set( MAX_NUM_ITERATIONS, 1 );
    solver.set( KKT_TOLERANCE, 1e-3 );
    solver.set( QP_SOLVER, QP_QPOASES );
    solver.set( HOTSTART_QP, YES );

    /*// Solve the MPC problem
    if (solver.solve() != SUCCESSFUL_RETURN) {
        std::cerr << "MPC failed to solve" << std::endl;
        return -1;
    }*/

    // Define the initial state, parameters, and reference trajectory
    //DVector X_MPC = (); // Initialize with the current state
    //DVector _p = ...; // Initialize parameters if any
    //VariablesGrid X_MPC_desired = (0,0,0,0,0,0,0,0,0,0,0); // Define the reference trajectory

    /*
    returnValue result = solver.solve(0.0, X_MPC, _p, _yRef);

    if (result != SUCCESSFUL_RETURN) {
        std::cerr << "MPC failed to solve" << std::endl;
        return; // Make sure your function return type matches this action
    }*/

    // Retrieve the control and state solution
    VariablesGrid state_trajectory, control_trajectory;
    solver.getDifferentialStates(state_trajectory);
    solver.getControls(control_trajectory);


    auto msg = std::make_unique<crazyflie_msgs::msg::AttitudeCommand>();

    msg->pitch = 0;
    msg->roll = 0;
    msg->thurst = 0;
    msg->yaw = 0;

    _pub_attutude_cmd->publish(std::move(msg));

    _old_time = now();
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionMPC>());
    rclcpp::shutdown();
    return 0;
}
