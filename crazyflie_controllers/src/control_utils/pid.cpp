#include "crazyflie_controllers/control_utils/pid.h"
#include <cmath>

PID::PID( double kp,  double ki,  double kd, bool debug, bool enable_low_pass_derivative, double low_pass_tau) :
    _kp(kp), _ki(ki),_kd(kd), _debug(debug), _prev_error(0), _prev_unfiltered_derivative(0), _enable_low_pass_derivative(enable_low_pass_derivative), 
    _tau_low_pass(low_pass_tau)
{
}

PID::PID( double kp,  double ki,  double kd, bool debug) :
    _kp(kp), _ki(ki),_kd(kd), _debug(debug), _prev_error(0), _prev_unfiltered_derivative(0), _enable_low_pass_derivative(false), 
    _tau_low_pass(0)
{
}
PID::PID( double kp,  double ki,  double kd) :
    _kp(kp), _ki(ki),_kd(kd), _debug(false), _prev_error(0), _prev_unfiltered_derivative(0), _enable_low_pass_derivative(false), 
    _tau_low_pass(0)
{
}

PID::PID( double kp,  double ki,  double kd, bool enable_low_pass_derivative, double low_pass_tau) :
    _kp(kp), _ki(ki),_kd(kd), _debug(false), _prev_error(0), _prev_unfiltered_derivative(0), _enable_low_pass_derivative(enable_low_pass_derivative), 
    _tau_low_pass(low_pass_tau)
{
}   

double PID::getCommand(const double value, const double target, const rclcpp::Duration dt) 
{       
    // P term
    double error_term = target - value;
    double pid_P = _kp * std::clamp(error_term, -1.0, 1.0);

    // TODO: integral term
    double pid_I = 0;

    // Derivative term
    double derivative = 0;
    double unfiltered_derivative = (error_term - _prev_error) / dt.seconds();
    _prev_error = error_term;

    // Either filter derivative or not filter
    if (!_enable_low_pass_derivative) {
        // don't apply low-pass filter
        derivative = unfiltered_derivative;
    } else {                    
        // First order derivative to mitigate derivative kick
        derivative = _prev_unfiltered_derivative + (_tau_low_pass / (dt.seconds() + _tau_low_pass)) * (unfiltered_derivative - _prev_unfiltered_derivative);
        _prev_unfiltered_derivative = unfiltered_derivative;
    }

    if(_debug)
    {
        std::cout << "error term: " << error_term << std::endl;
        std::cout << "derivative: " << derivative << std::endl;
    }

    double pid_D = _kd * std::clamp(derivative, -1.0, 1.0);

    double command = pid_P + pid_I + pid_D;
    
    return command;
}

void PID::update( double kp,  double ki,  double kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

Angular_PID::Angular_PID( double kp ,  double ki ,  double kd, bool debug ) :
    PID(kp, ki, kd, debug)
{

}

double Angular_PID::getCommand(const double value, const double target, const rclcpp::Duration dt)
{
        double abs_distance_error = fmod(target - value, 2.0 * M_PI);
        // solves problem in case for ex moving from -3.10 to 3.10
        double error_term = abs_distance_error < M_PI ? abs_distance_error : abs_distance_error - 2*M_PI;
        double pid_P = _kp * std::clamp(error_term, -1.0, 1.0);
        // integral part TODO
        double pid_I = 0;
        double dist_difference = (error_term - _prev_error) / dt.seconds();
        double pid_D = _kd * dist_difference;
        _prev_error = error_term;


        double command = pid_P + pid_I + pid_D;
        
        return command;
}

