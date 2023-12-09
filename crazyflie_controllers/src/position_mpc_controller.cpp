#include "crazyflie_controllers/position_mpc_controller.h"

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
            std::chrono::milliseconds(1000 / CONTROLLER_FREQ),
            std::bind(&PositionMPC::_sendCommandAttitude, this));

        _prev_time = now();
        _is_prev_time_position_set = false;
  
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
}

void PositionMPC::_newGpsCallback(const geometry_msgs::msg::PointStamped::SharedPtr gps_data)
{
    if(!_is_prev_time_position_set){
        _prev_time_position = now();
        _is_prev_time_position_set = true;

        //This will be set to compare to the next position to generate the velocity
        p_WB_W_prev = tf2::Vector3(gps_data->point.x, gps_data->point.y, gps_data->point.z);

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

    RCLCPP_INFO(this->get_logger(),
                 "GPS updated, p_WB.x: %f, p_WB.y(): %f, p_WB.z = %f", p_WB_W.x(), p_WB_W.y(), p_WB_W.z());
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


    RCLCPP_INFO(this->get_logger(),
                 "imu data received! [R_WB_roll: %f, R_WB_pitch: %f, R_WB_yaw: %f]",
                 roll, pitch, yaw);
}

void PositionMPC::_sendCommandAttitude()
{
    rclcpp::Duration dt = now() - _prev_time;

    // Constants
    const double Ix = 0.0142;  // Moment of inertia around p_WB_W_x_df-axis
    const double Iy = 0.0142;  // Moment of inertia around p_WB_W_y_df-axis
    const double Iz = 0.0284;  // Moment of inertia around p_WB_W_z_df-axis
    const double m = 1.56779;  // Mass of the quadrotor
    const double g = 9.81;     // Acceleration due to gravity
    const double samplingTime = 0.01; // Sampling time for real-time control

    // Initialize ACADO environment
    DifferentialState R_WB_roll_df, R_WB_pitch_df, R_WB_yaw_df; // Orientation (roll, pitch, yaw)
    DifferentialState omega_WB_roll_df, omega_WB_pitch_df, omega_WB_yaw_df; // Angular rates
    DifferentialState v_WB_x_df, v_WB_y_df, v_WB_z_df;         // Velocities
    DifferentialState p_WB_W_x_df, p_WB_W_y_df, p_WB_W_z_df;   // Position


    Control ft_df;                        // Thrust
    Control tau_x_df, tau_y_df, tau_z_df;       // Torques

    // Define the linearized dynamics of the quadrotor
    DifferentialEquation f;

    /*
    // Angular rates dynamics
    f << dot(R_WB_roll_df) == omega_WB_roll_df + omega_WB_yaw_df*R_WB_pitch_df;
    f << dot(R_WB_pitch_df) == omega_WB_pitch_df - omega_WB_yaw_df*R_WB_roll_df;
    f << dot(R_WB_yaw_df) == omega_WB_yaw_df + omega_WB_pitch_df*R_WB_roll_df;

    // Angular accelerations dynamics
    f << dot(omega_WB_roll_df) == (tau_x_df - (Iz - Iy)*omega_WB_yaw_df*omega_WB_pitch_df) / Ix;
    f << dot(omega_WB_pitch_df) == (tau_y_df - (Ix - Iz)*omega_WB_yaw_df*omega_WB_roll_df) / Iy;
    f << dot(omega_WB_yaw_df) == (tau_z_df - (Iy - Ix)*omega_WB_roll_df*omega_WB_pitch_df) / Iz;

    // Linear accelerations dynamics
    f << dot(v_WB_x_df) == omega_WB_yaw_df*v_WB_y_df - omega_WB_pitch_df*v_WB_z_df - g*R_WB_pitch_df + ft_df/m;
    f << dot(v_WB_y_df) == omega_WB_roll_df*v_WB_z_df - omega_WB_yaw_df*v_WB_x_df + g*R_WB_roll_df;
    f << dot(v_WB_z_df) == omega_WB_pitch_df*v_WB_x_df - omega_WB_roll_df*v_WB_y_df + g - ft_df/m;

    // Linear velocities dynamics
    f << dot(p_WB_W_x_df) == v_WB_x_df;
    f << dot(p_WB_W_y_df) == v_WB_y_df;
    f << dot(p_WB_W_z_df) == v_WB_z_df;
     */

    // Angular rates dynamics
    f << dot(R_WB_roll_df) == omega_WB_roll_df;
    f << dot(R_WB_pitch_df) == omega_WB_pitch_df;
    f << dot(R_WB_yaw_df) == omega_WB_yaw_df;

    // Angular accelerations dynamics
    f << dot(omega_WB_roll_df) == tau_x_df / Ix;
    f << dot(omega_WB_pitch_df) == tau_y_df / Iy;
    f << dot(omega_WB_yaw_df) == tau_z_df / Iz;

    // Linear accelerations dynamics
    f << dot(v_WB_x_df) == -g * R_WB_pitch_df;
    f << dot(v_WB_y_df) == g * R_WB_roll_df;
    f << dot(v_WB_z_df) == ft_df / m;

    // Linear velocities dynamics
    f << dot(p_WB_W_x_df) == v_WB_x_df;
    f << dot(p_WB_W_y_df) == v_WB_y_df;
    f << dot(p_WB_W_z_df) == v_WB_z_df;

    // Set up the MPC optimization problem
    // Prediction horizon of 2 seconds, with sampling time intervals
    OCP ocp(0.0, 2.0, (int)(2.0/samplingTime));

    Function h;
    h << p_WB_W_x_df << p_WB_W_y_df << p_WB_W_z_df << R_WB_roll_df << R_WB_pitch_df << R_WB_yaw_df << v_WB_x_df << v_WB_y_df << v_WB_z_df << omega_WB_roll_df << omega_WB_pitch_df << omega_WB_yaw_df;
    h << ft_df << tau_x_df << tau_y_df << tau_z_df;

    //DMatrix Q(h.getDim(), h.getDim());
    //Q.eye(); // Simple identity matrix for weighting
    //const int dim = h.getDim();
    //uint dim = static_cast<uint>(h.getDim());
    //DMatrix Q(dim, dim);
    //DMatrix Q(dim, dim);
    //Q.setIdentity();
    /* uint dim = static_cast<uint>(h.getDim());
    DMatrix Q(dim, dim); // Initializes a dim p_WB_W_x_df dim matrix with zeros

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
    ocp.subjectTo(-10.0 <= p_WB_W_x_df <= 10.0); // p_WB_W_x_df position constraint
    ocp.subjectTo(-10.0 <= p_WB_W_y_df <= 10.0); // p_WB_W_y_df position constraint
    ocp.subjectTo(-10.0 <= p_WB_W_z_df <= 10.0); // p_WB_W_z_df position constraint
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
    DVector X_MPC(12);

    double R_WB_roll, R_WB_pitch, R_WB_yaw;
    R_WB.getRPY(R_WB_roll, R_WB_pitch, R_WB_yaw);

    X_MPC(0) = R_WB_roll;
    X_MPC(1) = R_WB_pitch;
    X_MPC(2) = R_WB_yaw;
    X_MPC(3) = omega_WB.x();
    X_MPC(4) = omega_WB.y();
    X_MPC(5) = omega_WB.z();
    X_MPC(6) = v_WB.x();
    X_MPC(7) = v_WB.y();
    X_MPC(8) = v_WB.z();
    X_MPC(9) = p_WB_W.x();
    X_MPC(10) = p_WB_W.y();
    X_MPC(11) = p_WB_W.z();


    DVector _p; // Initialize parameters if any
    //VariablesGrid X_MPC_desired_trajectory = (0,0,0,0,0,0,0,0,0,0,0); // Define the reference trajectory

    // 12 state variables and 1 grid point, our target state
    VariablesGrid X_MPC_desired_trajectory(12, Grid(0.0, 2.0, 1));
    X_MPC_desired_trajectory(0, 0) = 0.0; // Target roll
    X_MPC_desired_trajectory(0, 1) = 0.0; // Target pitch
    X_MPC_desired_trajectory(0, 2) = 0.0; // Target yaw (changeable)
    X_MPC_desired_trajectory(0, 3) = 0.0; // Target angular roll velocity
    X_MPC_desired_trajectory(0, 4) = 0.0; // Target angular pitch velocity
    X_MPC_desired_trajectory(0, 5) = 0.0; // Target angular yaw velocity
    X_MPC_desired_trajectory(0, 6) = 0.0; // Target velocity in x-coordinate
    X_MPC_desired_trajectory(0, 7) = 0.0; // Target velocity in y-coordinate
    X_MPC_desired_trajectory(0, 8) = 0.0; // Target velocity in z-coordinate
    X_MPC_desired_trajectory(0, 9) = 0.0; // Target x-coordinate
    X_MPC_desired_trajectory(0, 10) = 0.0; // Target y-coordinate
    X_MPC_desired_trajectory(0, 11) = 1.0; // Target z-coordinate (1 meter height) hard-coded for now


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

    double thrust = control_trajectory(0, 0); // Access the optimized thrust value
    double tau_x = control_trajectory(0, 1); // Access the optimized torque around x-axis
    double tau_y = control_trajectory(0, 2); // Access the optimized torque around y-axis
    double tau_z = control_trajectory(0, 3); // Access the optimized torque around z-axis


    msg->pitch = tau_y;
    msg->roll = tau_x;
    msg->thurst = thrust;
    msg->yaw = tau_z;

    _pub_attutude_cmd->publish(std::move(msg));

    _prev_time = now();
}

int main(int argc, char const *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PositionMPC>());
    rclcpp::shutdown();
    return 0;
}
