#pragma once
#include "rclcpp/rclcpp.hpp"
#include <stdio.h>
#include <iostream>
#include <algorithm>

class PID {
public:
    PID( double kp = 0,  double ki = 0,  double kd = 0,  double freq=100);
    double getCommand(const double value, const double target, const rclcpp::Duration dt);
    void update( double kp = 0,  double ki = 0,  double kd = 0 );

private:
    double _kp;
    double _ki;
    double _kd;
    const double _freq;
    double _previous_distance_error;
    int _count;
};