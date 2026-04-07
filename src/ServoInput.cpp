#include "ServoInput.h"

ServoInput* ServoInput::_instances[30] = {nullptr};

ServoInput::ServoInput(int pin) : _pin(pin) {
    _instances[pin] = this;
}

void ServoInput::begin() {
    pinMode(_pin, INPUT_PULLUP);
    if (_pin == 0) {
        attachInterrupt(digitalPinToInterrupt(_pin), ServoInput::_isr_wrapper_0, CHANGE);
    } else if (_pin == 1) {
        attachInterrupt(digitalPinToInterrupt(_pin), ServoInput::_isr_wrapper_1, CHANGE);
    } else if (_pin == 2) {
        attachInterrupt(digitalPinToInterrupt(_pin), ServoInput::_isr_wrapper_2, CHANGE);
    } else if (_pin == 3) {
        attachInterrupt(digitalPinToInterrupt(_pin), ServoInput::_isr_wrapper_3, CHANGE);
    } else if (_pin == 4) {
        attachInterrupt(digitalPinToInterrupt(_pin), ServoInput::_isr_wrapper_4, CHANGE);
    } else if (_pin == 5) {
        attachInterrupt(digitalPinToInterrupt(_pin), ServoInput::_isr_wrapper_5, CHANGE);
    }
}

void ServoInput::_isr_wrapper_0() {
    if (_instances[0]) _instances[0]->_handle_interrupt();
}

void ServoInput::_isr_wrapper_1() {
    if (_instances[1]) _instances[1]->_handle_interrupt();
}

void ServoInput::_isr_wrapper_2() {
    if (_instances[2]) _instances[2]->_handle_interrupt();
}

void ServoInput::_isr_wrapper_3() {
    if (_instances[3]) _instances[3]->_handle_interrupt();
}

void ServoInput::_isr_wrapper_4() {
    if (_instances[4]) _instances[4]->_handle_interrupt();
}

void ServoInput::_isr_wrapper_5() {
    if (_instances[5]) _instances[5]->_handle_interrupt();
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
