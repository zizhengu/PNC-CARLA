#pragma once

#include<unordered_map>
#include"rclcpp/rclcpp.hpp"

class LongitudinalPIDController{
public:
    LongitudinalPIDController();
    // double run_step(const double &target_speed, const double &current_speed);
    double run_step_throttle(const double &target_speed, const double &current_speed);
    double run_step_brake(const double &target_speed, const double &current_speed);

private:
    std::unordered_map<std::string, double>
        _args_longitudinal;

    double _longitudinal_error_proportional;
    double _longitudinal_error_integral;
    double _longitudinal_error_derivative;
    double _longitudinal_previous_error_proportional;

};