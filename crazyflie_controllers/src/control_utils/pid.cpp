#include "crazyflie_controllers/control_utils/pid.h"
#include <cmath>

PID::PID( double kp,  double ki,  double kd, bool debug) :
    _kp(kp), _ki(ki),_kd(kd), _debug(debug)
{
}   

double PID::getCommand(const double value, const double target, const rclcpp::Duration dt) 
{         

         
        double distance_error = target - value;
        double pid_P = _kp * std::clamp(distance_error, -1.0, 1.0);

        
        
        // Then in your getCommand function
        //_integral_error += distance_error * dt.seconds();
        //double pid_I = _ki * _integral_error;
        double pid_I = 0;


         
        double dist_difference = (distance_error - _previous_distance_error) / dt.seconds();
        double pid_D = _kd * std::clamp(dist_difference, -1.0, 1.0);
        _previous_distance_error = distance_error;

        if (_debug) {
            std::cout << "inside pid calcu: " << "value: " << value << ", target: " << target << std::endl;
            std::cout << "resulting pid_P: " << pid_P << std::endl;
            std::cout << "resulting pid_D: " << pid_D << std::endl;
        }
        // if(_debug)
        // {
        //     std::cout << "dist_error: " << distance_error << ", previous_dist_error: " << _previous_distance_error << std::endl;
        //     std::cout << "pid_P: " << pid_P << ", pid_D: " << pid_D << ", pid_I: " << pid_I << std::endl; 
        //     std::cout << "dist_difference: " << dist_difference << "dt: " << dt.seconds() << std::endl; 
        //     std::cout << "value: " << value << ", target: " << target << std::endl; 
        //     std::cout << std::endl;
        // }

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

