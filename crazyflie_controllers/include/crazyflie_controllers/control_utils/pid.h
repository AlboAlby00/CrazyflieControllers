#pragma once
#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <algorithm>

class PID {
    public:
        PID( double kp,  double ki,  double kd, bool debug, bool enable_low_pass_derivative, double low_pass_tau);
        PID( double kp,  double ki,  double kd, bool debug);
        PID( double kp,  double ki,  double kd);
        PID( double kp,  double ki,  double kd, bool enable_low_pass_derivative, double low_pass_tau);
        double getCommand(const double value, const double target, const rclcpp::Duration dt);
        void update( double kp = 0,  double ki = 0,  double kd = 0 );

    protected:
        // Scaling factor for P term
        double _kp;
        // Scaling factor for integral term
        double _ki;
        // Scaling factor for derivative term
        double _kd;
        // Enable debug messages
        const bool _debug;
        // Computed error from previous timestep
        double _prev_error;
        // Computed derivative from previous timestep
        double _prev_unfiltered_derivative;
        // Enable low-pass filtering on derivative term to mitigate derivative kick
        bool _enable_low_pass_derivative;
        // Tau parameter for first order lowpass filter on derivative term
        double _tau_low_pass;
        int _count;
        double _integral_error;
};

class Angular_PID : public PID
{
    public:
        Angular_PID( double kp = 0,  double ki = 0,  double kd = 0, bool debug=false);
        double getCommand(const double value, const double target, const rclcpp::Duration dt);
};