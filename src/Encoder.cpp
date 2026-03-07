#include "Encoder.h"

Encoder* Encoder::_instances[30] = {nullptr};

Encoder::Encoder(int pin, int moving_avg_size) 
    : _pin(pin), _avg_size(moving_avg_size) {
    _history = new double[_avg_size];
    _instances[pin] = this;
}

void Encoder::begin() {
    pinMode(_pin, INPUT_PULLUP);
    _last_edge_time = micros();
    if (_pin == 2) {
        attachInterrupt(digitalPinToInterrupt(_pin), Encoder::_isr_wrapper_2, CHANGE);
    }
}

void Encoder::_isr_wrapper_2() {
    if (_instances[2]) _instances[2]->_handle_interrupt();
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

    double raw = (double)h * 360.0 / (double)period;
    
    if (!_initialized) {
        _last_raw = raw;
        _total_accumulated = raw;
        _start_offset = _total_accumulated;
        for(int i=0; i<_avg_size; i++) _history[i] = 0.0;
        _initialized = true;
        return true;
    }

    // --- Unwrap against the LAST VALID sample ---
    double delta = raw - _last_raw;
    
    // Shortest path logic
    if (delta > 180.0) delta -= 360.0;
    else if (delta < -180.0) delta += 360.0;

    // VALIDATION: Reject jumps that are physically impossible
    // At 244Hz, a 150-degree jump is ~6,250 RPM.
    // If a noise spike (like 180.0) appears, delta will be large and we reject it.
    // Crucially: we DO NOT update _last_raw, so the next real sample
    // will be compared against the last known-good position.
    if (abs(delta) > 150.0) {
        return false; 
    }

    _total_accumulated += delta;
    _last_raw = raw; // ONLY update reference if the sample was valid

    // --- Smoothing (Only on final result) ---
    double continuous_now = _total_accumulated - _start_offset;
    
    _history[_h_idx] = continuous_now;
    _h_idx = (_h_idx + 1) % _avg_size;
    if (_h_count < _avg_size) _h_count++;

    double sum = 0;
    for (int i = 0; i < _h_count; i++) sum += _history[i];
    _filtered_continuous = sum / _h_count;

    return true;
}
