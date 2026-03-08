#include "Encoder.h"

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
}

void Encoder::_handle_interrupt() {
    uint32_t now = micros();
    uint32_t duration = now - _last_edge_time;
    _last_edge_time = now;

    if (gpio_get(_pin)) {
        _last_low_us = duration;
        _data_ready = true; 
    } else {
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
    if (period < 100) return false;

    _last_period = period;
    double raw = (double)h * 360.0 / (double)period;
    
    if (!_initialized) {
        _last_raw = raw;
        _total_accumulated = raw;
        _start_offset = _total_accumulated;
        _filtered_continuous = 0.0;
        _initialized = true;
        return true;
    }

    // 1. Unwrap against the LAST VALID sample
    double delta = raw - _last_raw;
    if (delta > 180.0) delta -= 360.0;
    else if (delta < -180.0) delta += 360.0;

    // 2. Velocity Cap Validation
    if (abs(delta) > 150.0) {
        return false; 
    }

    _total_accumulated += delta;
    _last_raw = raw;

    // 3. One-Pole Low-Pass Filter on the continuous result
    double current_continuous = _total_accumulated - _start_offset;
    _filtered_continuous = (_alpha * current_continuous) + ((1.0f - _alpha) * _filtered_continuous);

    return true;
}
