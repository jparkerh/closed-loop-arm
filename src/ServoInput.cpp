#include "ServoInput.h"

ServoInput* ServoInput::_instances[30] = {nullptr};

ServoInput::ServoInput(int pin) : _pin(pin) {
    _instances[pin] = this;
}

void ServoInput::begin() {
    pinMode(_pin, INPUT_PULLUP);
    if (_pin == 2) {
        attachInterrupt(digitalPinToInterrupt(_pin), ServoInput::_isr_wrapper_2, CHANGE);
    }
}

void ServoInput::_isr_wrapper_2() {
    if (_instances[2]) _instances[2]->_handle_interrupt();
}

void ServoInput::_handle_interrupt() {
    uint32_t now = micros();
    if (gpio_get(_pin)) {
        _pulse_start_us = now;
    } else {
        if (_pulse_start_us != 0) {
            uint32_t width = now - _pulse_start_us;
            // Validate: check against standard servo signal range (500-2500us)
            if (width >= 400 && width <= 2600) {
                _last_pulse_width_us = width;
                _last_pulse_time_ms = millis();
                _new_data = true;
            }
        }
    }
}

bool ServoInput::hasNewData() {
    bool state = _new_data;
    _new_data = false;
    return state;
}
