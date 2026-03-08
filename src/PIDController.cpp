#include "PIDController.h"

PIDController::PIDController(double kp, double ki, double kd) 
    : _kp(kp), _ki(ki), _kd(kd) {
    _min_output = -1000.0;
    _max_output = 1000.0;
    _integral_limit = 500.0;
}

void PIDController::setGains(double kp, double ki, double kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::setOutputLimits(double min, double max) {
    _min_output = min;
    _max_output = max;
}

void PIDController::setIntegralLimit(double limit) {
    _integral_limit = limit;
}

void PIDController::reset() {
    _integral = 0;
    _last_error = 0;
    _first_run = true;
}

double PIDController::compute(double target, double current, double dt) {
    if (dt <= 0) return 0;
    
    double error = target - current;

    // Proportional
    double p_out = _kp * error;

    // Integral with anti-windup clamping
    _integral += error * dt;
    _integral = constrain(_integral, -_integral_limit, _integral_limit);
    double i_out = _ki * _integral;

    // Derivative
    double d_out = 0;
    if (!_first_run) {
        d_out = _kd * (error - _last_error) / dt;
    }
    _first_run = false;
    _last_error = error;

    // Final output with total clamping
    double output = p_out + i_out + d_out;
    return constrain(output, _min_output, _max_output);
}
