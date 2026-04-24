#include <Arduino.h>
#include <Servo.h>
#include <hardware/watchdog.h>
#include "Encoder.h"
#include "ServoInput.h"
#include "PIDController.h"

// Hardware Configuration
const int ENCODER_PIN = 29;
const int SERVO_IN_PIN = 2;
const int OVERRIDE_PIN = 3;
const int MANUAL_PIN = 5;
const int SERVO_OUT_PIN = 4;

// UART Configuration (GP0=TX, GP1=RX)
#define UART_TX_PIN 0
#define UART_RX_PIN 1

// --- Atomic Shared State ---
volatile float shared_kp = 5.0f;
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
volatile uint32_t shared_overrideUs = 1500;
volatile uint32_t shared_manualUs = 1500;

volatile bool heartbeat0 = false;
volatile bool heartbeat1 = false;

// Global objects
Encoder encoder(ENCODER_PIN, 0.5f);
ServoInput servoIn(SERVO_IN_PIN);
ServoInput overrideIn(OVERRIDE_PIN);
ServoInput manualIn(MANUAL_PIN);
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

    static bool wasOverride = false;
    bool isOverride = (shared_overrideUs > 1600);

    if (isOverride) {
        // MANUAL OVERRIDE: Direct pass-through of manual stick
        finalOut = shared_manualUs;
        if (finalOut < 800 || finalOut > 2200) finalOut = 1500; // Failsafe clamp
        wasOverride = true;
    } else {
        // Transition Detection: Just entered Auto mode from Manual
        if (wasOverride) {
            encoder.resetZero();
            pid.reset();
            wasOverride = false;
        }

        if (shared_systemEnabled && age < 100) {
            // CLOSED LOOP: PID Control
            finalOut = currentPIDOutputUs;
        }
    }
    
    motorOutput.writeMicroseconds(finalOut);
    shared_finalOutputUs = finalOut;
}

// ================================================================
// CORE 1: I/O, Watchdog Feeding, and Interface
// ================================================================
void setup1() {
    Serial.begin(115200);
    
    // Dedicated Hardware UART for Telemetry (GP0=TX, GP1=RX)
    Serial1.setTX(UART_TX_PIN);
    Serial1.setRX(UART_RX_PIN);
    Serial1.begin(115200);

    servoIn.begin();
    overrideIn.begin();
    manualIn.begin();
    watchdog_enable(200, 1); 
}

void processCommands() {
    // Process commands from both USB and Hardware UART
    Stream* inputs[] = {&Serial, &Serial1};
    for (Stream* s : inputs) {
        if (s->available()) {
            char cmd = s->read();
            float val = s->parseFloat();
            while (s->available() && s->peek() < 'A') s->read();

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

    overrideIn.hasNewData();
    shared_overrideUs = overrideIn.getPulseWidthUs();

    manualIn.hasNewData();
    shared_manualUs = manualIn.getPulseWidthUs();
    
    if (shared_signalAge < 200) {
        shared_targetNorm = (float)(1500 - (int32_t)inputUs) / 1000.0f;
    } else {
        shared_targetNorm = 0; 
    }

    if (!hasZeroed && millis() > 1000) {
        encoder.resetZero();
        shared_systemEnabled = true; 
        hasZeroed = true;
        Serial.println(">>> READY: Calibration Complete <<<");
    }
// 4. Telemetry (50ms) - Only output AFTER warmup is complete
static uint32_t lastReportMs = 0;
    if (hasZeroed && millis() - lastReportMs > 50) {
        char statusStr[5];
        if (shared_overrideUs > 1600) {
            strcpy(statusStr, "MAN");
        } else if (shared_systemEnabled) {
            strcpy(statusStr, "RUN");
        } else {
            strcpy(statusStr, "STOP");
        }

        char buf[256];
        snprintf(buf, sizeof(buf), 
                 "Tgt:%5.2f | Cur:%5.2f | Eff:%5.2f | Out:%4lu | Stk:%4lu | Ovr:%4lu | Man:%4lu | Age:[%3lu,%3lu] | Status:%s", 
                 shared_targetNorm, shared_currentNorm, shared_effort, 
                 shared_finalOutputUs, inputUs, shared_overrideUs, shared_manualUs, 
                 shared_signalAge, shared_encoderAge,
                 statusStr);
        Serial.println(buf);
        Serial1.println(buf); // Mirror to Datalogging port
        lastReportMs = millis();
    }}
