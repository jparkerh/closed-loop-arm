#include <Arduino.h>
#include <Servo.h>
#include <hardware/watchdog.h>
#include "Encoder.h"
#include "ServoInput.h"
#include "PIDController.h"

// Hardware Configuration
const int ENCODER_PIN = 29;
const int SERVO_IN_PIN = 2;
const int SERVO_OUT_PIN = 4;

// --- Atomic Shared State ---
volatile float shared_kp = 10.0f;
volatile float shared_ki = 0.0f;
volatile float shared_kd = 0.0f;
volatile float shared_targetRange = 1080.0f; 
volatile bool shared_systemEnabled = false; 

volatile float shared_targetNorm = 0;
volatile float shared_currentNorm = 0;
volatile float shared_effort = 0;
volatile uint32_t shared_finalOutputUs = 1500;
volatile uint32_t shared_signalAge = 0;
volatile uint32_t shared_encoderAge = 0;

volatile bool heartbeat0 = false;
volatile bool heartbeat1 = false;

// Global objects
Encoder encoder(ENCODER_PIN, 0.5f);
ServoInput servoIn(SERVO_IN_PIN);
PIDController pid(0, 0, 0); 
Servo motorOutput; // Using the Servo library

bool hasZeroed = false;

// ================================================================
// CORE 0: Encoder Sync, PID Control & EXCLUSIVE ACTUATION
// ================================================================
void setup() {
    pinMode(LED_BUILTIN, OUTPUT);
    encoder.begin();
    
    // Core 0 Exclusive owner of the Servo
    motorOutput.attach(SERVO_OUT_PIN, 1000, 2000);
    motorOutput.writeMicroseconds(1500);
}

void loop() {
    static uint32_t lastEncoderUpdate = millis();
    static uint32_t lastBlink = 0;
    static uint32_t currentPIDOutputUs = 1500;
    heartbeat0 = true;

    // 1. PID Calculation
    if (encoder.update()) {
        uint32_t now = millis();
        float dt = (float)(now - lastEncoderUpdate) / 1000.0f;
        lastEncoderUpdate = now;
        shared_encoderAge = 0;

        float currentNorm = (float)encoder.getContinuousAngle() / shared_targetRange;
        pid.setGains(shared_kp, shared_ki, shared_kd);
        
        float effort = pid.compute(shared_targetNorm, currentNorm, dt);
        effort = -effort; 
        
        uint32_t outUs = 1500 + (int32_t)(effort * 500.0f);
        if (outUs < 1000) outUs = 1000;
        if (outUs > 2000) outUs = 2000;

        currentPIDOutputUs = outUs;
        shared_currentNorm = currentNorm;
        shared_effort = effort;
    }

    // 2. Consistent Servo Actuation
    uint32_t finalOut = 1500;
    uint32_t age = millis() - lastEncoderUpdate;
    shared_encoderAge = age;

    if (shared_systemEnabled && age < 100) {
        finalOut = currentPIDOutputUs;
    }
    
    motorOutput.writeMicroseconds(finalOut);
    shared_finalOutputUs = finalOut;

    // 3. Status LED
    uint32_t blinkInterval = 250 - (uint32_t)(abs(shared_effort) * 200.0f);
    if (millis() - lastBlink > blinkInterval) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        lastBlink = millis();
    }
}

// ================================================================
// CORE 1: I/O, Watchdog Feeding, and Interface
// ================================================================
void setup1() {
    Serial.begin(115200);
    servoIn.begin();
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
            case 'S':
                shared_systemEnabled = (val > 0.5f);
                if (!shared_systemEnabled) pid.reset();
                break;
        }
    }
}

void loop1() {
    heartbeat1 = true;
    if (heartbeat0 && heartbeat1) {
        watchdog_update();
        heartbeat0 = false;
        heartbeat1 = false;
    }

    processCommands();

    servoIn.hasNewData();
    uint32_t inputUs = servoIn.getPulseWidthUs();
    shared_signalAge = servoIn.getSignalAgeMs();
    
    if (shared_signalAge < 200) {
        shared_targetNorm = (float)(1500 - (int32_t)inputUs) / 1000.0f;
    } else {
        shared_targetNorm = 0; 
    }

    if (!hasZeroed && millis() > 5000) {
        encoder.resetZero();
        shared_systemEnabled = true; 
        hasZeroed = true;
        Serial.println(">>> READY: Calibration Complete <<<");
    }

    static uint32_t lastReportMs = 0;
    if (hasZeroed && millis() - lastReportMs > 50) {
        char buf[200];
        snprintf(buf, sizeof(buf), 
                 "Target: %5.2f | Current: %5.2f | Effort: %5.2f | Out: %4lu us | Age(ms): [Cmd:%3lu, Enc:%3lu] | Rej: %4lu | Gain: [P:%4.1f, I:%4.2f, D:%4.2f]", 
                 shared_targetNorm, shared_currentNorm, shared_effort, 
                 shared_finalOutputUs, shared_signalAge, shared_encoderAge,
                 encoder.getRejectionCount(),
                 shared_kp, shared_ki, shared_kd);
        Serial.println(buf);
        lastReportMs = millis();
    }
}
