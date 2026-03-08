#include "PIDController.h"

PIDController::PIDController(float kp, float ki, float kd) 
    : _kp(kp), _ki(ki), _kd(kd) {
    _min_output = -1.0f;
    _max_output = 1.0f;
    _integral_limit = 0.5f;
}

void PIDController::setGains(float kp, float ki, float kd) {
    _kp = kp;
    _ki = ki;
    _kd = kd;
}

void PIDController::setOutputLimits(float min, float max) {
    _min_output = min;
    _max_output = max;
}

void PIDController::setIntegralLimit(float limit) {
    _integral_limit = limit;
}

void PIDController::reset() {
    _integral = 0;
    _last_error = 0;
    _first_run = true;
}

float PIDController::compute(float target, float current, float dt) {
    if (dt <= 0) return 0;
    
    float error = target - current;

    // Proportional
    float p_out = _kp * error;

    // Integral with anti-windup
    _integral += error * dt;
    _integral = constrain(_integral, -_integral_limit, _integral_limit);
    float i_out = _ki * _integral;

    // Derivative
    float d_out = 0;
    if (!_first_run) {
        d_out = _kd * (error - _last_error) / dt;
    }
    _first_run = false;
    _last_error = error;

    float output = p_out + i_out + d_out;
    return constrain(output, _min_output, _max_output);
}
