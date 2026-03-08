#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
public:
    Encoder(int pin, float alpha = 0.5f);
    void begin();
    
    // Call this in the main loop to process new pulses
    bool update();
    
    // Resets the zero point to the current position
    void resetZero();
    
    // Setter for filter smoothing (0.0 to 1.0)
    void setAlpha(float alpha) { _alpha = alpha; }
    
    double getRawAngle() const { return _last_raw; }
    double getContinuousAngle() const { return _filtered_continuous; }
    uint32_t getPeriod() const { return _last_period; }

private:
    int _pin;
    float _alpha;
    
    // Measurement State (volatile for ISR)
    static Encoder* _instances[30];
    volatile uint32_t _last_high_us = 0;
    volatile uint32_t _last_low_us = 0;
    volatile uint32_t _last_edge_time = 0;
    volatile bool _data_ready = false;

    // Processing State
    double _last_raw = 0.0;
    double _total_accumulated = 0.0;
    double _start_offset = 0.0;
    bool _initialized = false;
    uint32_t _last_period = 0;

    // Filter State (One-Pole Low-Pass)
    double _filtered_continuous = 0.0;

    static void _isr_wrapper_2(); 
    static void _isr_wrapper_28(); 
    static void _isr_wrapper_29(); 
    void _handle_interrupt();
};

#endif
