#include "crazyflie_controllers/control_utils/pid.h"
#include <cmath>

PID::PID( double kp,  double ki,  double kd, bool debug) :
    _kp(kp), _ki(ki),_kd(kd), _debug(debug)
{
    this->_previous_distance_error = 0;
    this->_previous_filtered_error = 0;
}   

double PID::getCommand(const double value, const double target, const rclcpp::Duration dt, bool use_low_pass_filter, double low_pass_filter_tau) 
{         

         
        double distance_error = target - value;
        double pid_P = _kp * std::clamp(distance_error, -1.0, 1.0);


        double pid_I = 0;
        double dist_difference = 0;
        if (!use_low_pass_filter) {
            dist_difference = (distance_error - _previous_distance_error) / dt.seconds();
            _previous_distance_error = distance_error;
            
        } else {
            
            dist_difference = (distance_error - _previous_distance_error) / dt.seconds();
            double filtered_error = _previous_filtered_error + (low_pass_filter_tau / (dt.seconds() + low_pass_filter_tau)) * (dist_difference - _previous_filtered_error);

            // 3. Update the previous values for the next iteration
            _previous_distance_error = distance_error;
            _previous_filtered_error = dist_difference;
            dist_difference = filtered_error;
        }
                 
        // double dist_difference = (distance_error - _previous_distance_error) / dt.seconds();
        double pid_D = _kd * std::clamp(dist_difference, -1.0, 1.0);


        double command = pid_P + pid_I + pid_D; // Why do you add them up together?
        
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
        double distance_error = abs_distance_error < M_PI ? abs_distance_error : abs_distance_error - 2*M_PI;
        double pid_P = _kp * std::clamp(distance_error, -1.0, 1.0);
        // integral part TODO
        double pid_I = 0;
        double dist_difference = (distance_error - _previous_distance_error) / dt.seconds();
        double pid_D = _kd * dist_difference;
        _previous_distance_error = distance_error;

        // if(_debug)
        // {
        //     std::cout << "dist_error: " << distance_error << ", previous_dist_error: " << _previous_distance_error << std::endl;
        //     std::cout << "pid_P: " << pid_P << ", pid_D: " << pid_D << ", pid_I: " << pid_I << std::endl;
        //     std::cout << "dist_difference: " << dist_difference << "dt: " << dt.seconds() << std::endl;
        //     std::cout << "value: " << value << ", target: " << target << std::endl;
        //     std::cout << std::endl;
        // }

        double command = pid_P + pid_I + pid_D;
        
        return command;
}

