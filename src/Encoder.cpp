#include "Encoder.h"
#include <algorithm>

Encoder* Encoder::_instances[30] = {nullptr};

Encoder::Encoder(int pin, float alpha) 
    : _pin(pin), _alpha(alpha) {
    _instances[pin] = this;
}

void Encoder::begin() {
    pinMode(_pin, INPUT_PULLUP);
    _last_edge_time = micros();
    if (_pin == 2) {
        attachInterrupt(digitalPinToInterrupt(_pin), Encoder::_isr_wrapper_2, CHANGE);
    } else if (_pin == 28) {
        attachInterrupt(digitalPinToInterrupt(_pin), Encoder::_isr_wrapper_28, CHANGE);
    } else if (_pin == 29) {
        attachInterrupt(digitalPinToInterrupt(_pin), Encoder::_isr_wrapper_29, CHANGE);
    }
}

void Encoder::_isr_wrapper_2() {
    if (_instances[2]) _instances[2]->_handle_interrupt();
}

void Encoder::_isr_wrapper_28() {
    if (_instances[28]) _instances[28]->_handle_interrupt();
}

void Encoder::_isr_wrapper_29() {
    if (_instances[29]) _instances[29]->_handle_interrupt();
}

void Encoder::resetZero() {
    _initialized = false;
    _last_quadrant = -1;
    _total_turns = 0;
}

void Encoder::_handle_interrupt() {
    uint32_t now = micros();
    uint32_t duration = now - _last_edge_time;
    _last_edge_time = now;

    if (gpio_get(_pin)) {
        // High: The duration was the LOW phase.
        _last_low_us = duration;
        _data_ready = true; 
    } else {
        // Low: The duration was the HIGH phase.
        _last_high_us = duration;
    }
}

bool Encoder::update() {
    if (!_data_ready) return false;

    uint32_t h, l;
    noInterrupts();
    h = _last_high_us;
    l = _last_low_us;
    _data_ready = false;
    interrupts();

    uint32_t period = h + l;
    
    if (period < 500 || period > 2500) {
        if (_rejection_count % 100 == 0) {
            Serial.print("Rejected Period: ");
            Serial.println(period);
        }
        _rejection_count++;
        return false;
    }

    _last_period = period;
    double raw = (double)h * 360.0 / (double)period;
    
    int current_quadrant = (int)(raw / 90.0);
    if (current_quadrant > 3) current_quadrant = 3;

    if (!_initialized) {
        _last_raw = raw;
        _last_quadrant = current_quadrant;
        _total_turns = 0;
        _start_offset = raw;
        _filtered_continuous = 0.0;
        _initialized = true;
        return true;
    }

    // --- Quadrant-Based Unwrapping ---
    if (current_quadrant != _last_quadrant) {
        if (_last_quadrant == 3 && current_quadrant == 0) {
            _total_turns++; 
        } else if (_last_quadrant == 0 && current_quadrant == 3) {
            _total_turns--; 
        } else if (abs(current_quadrant - _last_quadrant) > 1) {
            if (_rejection_count % 100 == 0) {
                Serial.print("Rejected Jump: ");
                Serial.print(_last_quadrant);
                Serial.print(" -> ");
                Serial.println(current_quadrant);
            }
            _rejection_count++;
            return false; 
        }
        _last_quadrant = current_quadrant;
    }

    _last_raw = raw;
    double current_continuous = (_total_turns * 360.0) + raw - _start_offset;

    _filtered_continuous = (_alpha * current_continuous) + ((1.0f - _alpha) * _filtered_continuous);

    return true;
}
