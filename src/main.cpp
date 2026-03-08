#include <Arduino.h>
#include <hardware/pwm.h>
#include <hardware/watchdog.h>
#include "Encoder.h"
#include "ServoInput.h"
#include "PIDController.h"

// Hardware Configuration
const int ENCODER_PIN = 29;
const int SERVO_IN_PIN = 2;
const int SERVO_OUT_PIN = 4;

// Global objects
Encoder encoder(ENCODER_PIN, 0.5f);
ServoInput servoIn(SERVO_IN_PIN);
PIDController pid(7.0f, 0.0f, 0.05f);

// --- Atomic Shared State ---
volatile float shared_targetNorm = 0;
volatile float shared_currentNorm = 0;
volatile float shared_effort = 0;
volatile uint32_t shared_finalOutputUs = 1500;
volatile uint32_t shared_signalAge = 0;
volatile uint32_t shared_encoderAge = 0;

// Heartbeats for Hardware Watchdog
volatile bool heartbeat0 = false;
volatile bool heartbeat1 = false;

// Configurable Parameters
volatile float shared_kp = 10.0f;
volatile float shared_ki = 0.0f;
volatile float shared_kd = 0.05f;
volatile float shared_targetRange = 360.0f * 3.0f; 
volatile bool shared_systemEnabled = false; 

// Internal State
bool hasZeroed = false;
volatile uint32_t shared_manualOverrideUs = 0;
volatile uint32_t shared_manualOverrideEnd = 0;

// ================================================================
// HARDWARE PWM UTILITY
// ================================================================
void setServoHardwarePWM(uint pin, uint32_t width_us) {
    static bool initialized = false;
    uint slice = pwm_gpio_to_slice_num(pin);
    if (!initialized) {
        gpio_set_function(pin, GPIO_FUNC_PWM);
        pwm_set_clkdiv(slice, 125.0f); 
        pwm_set_wrap(slice, 20000);
        pwm_set_enabled(slice, true);
        initialized = true;
    }
    pwm_set_chan_level(slice, pwm_gpio_to_channel(pin), width_us);
}

// ================================================================
// CORE 0: Encoder Sync, PID Control & ACTUATION
// ================================================================
void setup() {
    encoder.begin();
    setServoHardwarePWM(SERVO_OUT_PIN, 1500);
}

void loop() {
    static uint32_t lastEncoderUpdate = millis();
    
    // Feed Heartbeat 0
    heartbeat0 = true;

    // 1. Synchronized PID Update + Direct Actuation
    if (encoder.update()) {
        uint32_t now = millis();
        float dt = (float)(now - lastEncoderUpdate) / 1000.0f;
        lastEncoderUpdate = now;
        shared_encoderAge = 0;

        float current = (float)encoder.getContinuousAngle();
        float range = shared_targetRange;
        float currentNorm = current / range;
        
        pid.setGains(shared_kp, shared_ki, shared_kd);
        float effort = pid.compute(shared_targetNorm, currentNorm, dt);
        
        uint32_t outUs = 1500 + (int32_t)(effort * 500.0f);
        if (outUs < 1000) outUs = 1000;
        if (outUs > 2000) outUs = 2000;

        uint32_t finalOut = 1500;
        if (millis() < shared_manualOverrideEnd) {
            finalOut = shared_manualOverrideUs;
        } else if (shared_systemEnabled) {
            finalOut = outUs;
        }
        
        setServoHardwarePWM(SERVO_OUT_PIN, finalOut);

        shared_currentNorm = currentNorm;
        shared_effort = effort;
        shared_finalOutputUs = finalOut;
    }

    // 2. Local Safety Watchdog (Immediate neutral if encoder lost)
    static uint32_t lastWatchdogCheck = 0;
    if (millis() - lastWatchdogCheck > 10) {
        uint32_t age = millis() - lastEncoderUpdate;
        shared_encoderAge = age;
        if (age > 50) {
            setServoHardwarePWM(SERVO_OUT_PIN, 1500);
            pid.reset();
        }
        lastWatchdogCheck = millis();
    }
}

// ================================================================
// CORE 1: I/O, Watchdog Feeding, and Interface
// ================================================================
void setup1() {
    Serial.begin(115200);
    servoIn.begin();
    
    // Enable Hardware Watchdog (200ms timeout)
    // We wait until setup is mostly done to avoid early resets.
    watchdog_enable(200, 1); 
}

void processCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        float val = Serial.parseFloat();
        while (Serial.available() && Serial.peek() < 'A') Serial.read();

        switch (cmd) {
            case 'P': shared_kp = val; break;
            case 'I': shared_ki = val; break;
            case 'D': shared_kd = val; break;
            case 'R': shared_targetRange = val; break;
            case 'M': 
                shared_manualOverrideUs = (uint32_t)val; 
                shared_manualOverrideEnd = millis() + 2000;
                break;
            case 'S':
                shared_systemEnabled = (val > 0.5f);
                if (!shared_systemEnabled) pid.reset();
                break;
        }
    }
}

void loop1() {
    // 1. Hardware Watchdog Feeding
    heartbeat1 = true;
    if (heartbeat0 && heartbeat1) {
        watchdog_update();
        heartbeat0 = false;
        heartbeat1 = false;
    }

    processCommands();

    // 2. Signal I/O
    servoIn.hasNewData();
    uint32_t inputUs = servoIn.getPulseWidthUs();
    shared_signalAge = servoIn.getSignalAgeMs();
    
    if (shared_signalAge < 200) {
        shared_targetNorm = (float)(1500 - (int32_t)inputUs) / 1000.0f;
    } else {
        shared_targetNorm = 0; 
    }

    // 3. Lifecycle
    if (!hasZeroed && millis() > 5000) {
        encoder.resetZero();
        shared_systemEnabled = true; 
        hasZeroed = true;
        Serial.println(">>> READY: Watchdog Enabled, System Live <<<");
    }

    // 4. Telemetry (50ms)
    static uint32_t lastReportMs = 0;
    if (millis() - lastReportMs > 50) {
        char buf[128];
        snprintf(buf, sizeof(buf), 
                 "T:%5.2f|C:%5.2f|E:%5.2f|O:%4lu|Age:%2lu,%2lu|%s", 
                 shared_targetNorm, shared_currentNorm, shared_effort, 
                 shared_finalOutputUs, shared_signalAge, shared_encoderAge,
                 (!hasZeroed) ? "WARM" : (shared_systemEnabled ? "RUN" : "STOP"));
        Serial.println(buf);
        lastReportMs = millis();
    }
}
