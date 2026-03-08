#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

class PIDController {
public:
    PIDController(double kp, double ki, double kd);
    
    // Setters for tuning
    void setGains(double kp, double ki, double kd);
    void setOutputLimits(double min, double max);
    void setIntegralLimit(double limit);
    
    // Core calculation
    double compute(double target, double current, double dt);
    
    // Reset internal state (integral and last error)
    void reset();

    // Getters for telemetry
    double getKp() const { return _kp; }
    double getKi() const { return _ki; }
    double getKd() const { return _kd; }

private:
    double _kp, _ki, _kd;
    double _min_output, _max_output;
    double _integral_limit;
    
    double _integral = 0;
    double _last_error = 0;
    bool _first_run = true;
};

#endif
