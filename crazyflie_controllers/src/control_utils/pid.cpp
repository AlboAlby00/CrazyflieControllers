#include "crazyflie_controllers/control_utils/pid.h"



PID::PID( double kp,  double ki,  double kd,  double freq) :
    _kp(kp), _ki(ki),_kd(kd), _freq(freq)
{
}   

double PID::getCommand(const double value, const double target, const rclcpp::Duration dt) 
{
        double distance_error = target - value;
        double pid_P = _kp * std::clamp(distance_error, -1.0, 1.0);

        // integral part TODO
        double pid_I = 0;

         
        double dist_difference = (distance_error - _previous_distance_error) / dt.seconds();
        double pid_D = _kd * dist_difference;
        _previous_distance_error = distance_error;

        //std::cout << "dist_error: " << distance_error << ", previous_dist_error: " << _previous_distance_error << std::endl;
        //std::cout << "pid_P: " << pid_P << ", pid_D: " << pid_D << std::endl; 
        //std::cout << "dist_difference: " << dist_difference << "dt: " << dt.seconds() << std::endl; 
        //std::cout << "value: " << value << "target: " << target << std::endl; 

        double command = pid_P + pid_I + pid_D;
        
        return command;
}

void PID::update( double kp,  double ki,  double kd)
{
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

