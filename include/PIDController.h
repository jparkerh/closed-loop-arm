#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
    PIDController(float kp, float ki, float kd);
    
    void setGains(float kp, float ki, float kd);
    void setOutputLimits(float min, float max);
    void setIntegralLimit(float limit);
    
    float compute(float target, float current, float dt);
    void reset();

    float getKp() const { return _kp; }
    float getKi() const { return _ki; }
    float getKd() const { return _kd; }

private:
    float _kp, _ki, _kd;
    float _min_output, _max_output;
    float _integral_limit;
    
    float _integral = 0;
    float _last_error = 0;
    bool _first_run = true;
};

#endif
