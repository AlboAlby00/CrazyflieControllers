#pragma once
#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <math.h>
#include <iostream>
#include <algorithm>

class PID {
    public:
        PID( double kp = 0,  double ki = 0,  double kd = 0, bool debug = false);
        double getCommand(const double value, const double target, const rclcpp::Duration dt);
        void update( double kp = 0,  double ki = 0,  double kd = 0 );

    protected:
        double _kp;
        double _ki;
        double _kd;
        const bool _debug;
        double _previous_distance_error;
        int _count;
        double _integral_error;
};

class Angular_PID : public PID
{
    public:
        Angular_PID( double kp = 0,  double ki = 0,  double kd = 0, bool debug=false);
        double getCommand(const double value, const double target, const rclcpp::Duration dt);
};