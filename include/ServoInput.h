#ifndef SERVO_INPUT_H
#define SERVO_INPUT_H

#include <Arduino.h>

class ServoInput {
public:
    ServoInput(int pin);
    void begin();
    
    // Returns the last valid pulse width in microseconds
    uint32_t getPulseWidthUs() const { return _last_pulse_width_us; }
    
    // Returns true if a new pulse has been processed since the last check
    bool hasNewData();

    // Returns the time in ms since the last valid pulse
    uint32_t getSignalAgeMs() const { return millis() - _last_pulse_time_ms; }

private:
    int _pin;
    volatile uint32_t _pulse_start_us = 0;
    volatile uint32_t _last_pulse_width_us = 0;
    volatile uint32_t _last_pulse_time_ms = 0;
    volatile bool _new_data = false;

    static ServoInput* _instances[30];
    static void _isr_wrapper_2(); // Specifically for GP2
    void _handle_interrupt();
};

#endif
