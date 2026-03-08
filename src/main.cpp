#include <Arduino.h>
#include <Servo.h>
#include "Encoder.h"
#include "ServoInput.h"

// Hardware Configuration
const int ENCODER_PIN = 29;
const int SERVO_IN_PIN = 2;
const int SERVO_OUT_PIN = 4;

// Global objects
Encoder encoder(ENCODER_PIN);
ServoInput servoIn(SERVO_IN_PIN);
Servo servoOut;

// Control Logic State
double targetRange = 2000.0; // +/- 2000 degrees maps to +/- 1.0
double kp = 2.0;
bool systemEnabled = false;

void setup() {
    Serial.begin(115200);
    unsigned long start_wait = millis();
    while (!Serial && millis() - start_wait < 3000); 
    
    Serial.println("\n--- Normalized Proportional Controller ---");
    Serial.println("Commands: P<val> (Gain), R<val> (MaxDeg), S0 (Stop), S1 (Start)");
    
    encoder.begin();
    servoIn.begin();
    servoOut.attach(SERVO_OUT_PIN, 500, 2500);
}

void processCommands() {
    if (Serial.available()) {
        char cmd = Serial.read();
        double val = Serial.parseFloat();
        while (Serial.available() && Serial.peek() < 'A') Serial.read();

        switch (cmd) {
            case 'P': kp = val; break;
            case 'R': targetRange = val; break;
            case 'S':
                systemEnabled = (val > 0.5);
                Serial.println(systemEnabled ? "SYSTEM ENABLED" : "SYSTEM DISABLED");
                break;
        }
    }
}

unsigned long lastReport = 0;

void loop() {
    processCommands();

    encoder.update();
    servoIn.hasNewData();

    uint32_t inputUs = servoIn.getPulseWidthUs();
    double targetNorm = 0;
    double currentNorm = 0;
    double effort = 0;
    uint32_t outputUs = 1500;

    // 1. Normalize Input: Map 500-2500us to 1.0 to -1.0 (flipped polarity)
    if (inputUs >= 400 && inputUs <= 2600) {
        targetNorm = (double)(1500 - (int32_t)inputUs) / 1000.0;
    }

    // 2. Normalize Feedback: Map ContinuousAngle to -1.0 to 1.0
    currentNorm = encoder.getContinuousAngle() / targetRange;

    // 3. Proportional Control on (-1, 1) scale
    double error = targetNorm - currentNorm;
    effort = -error * kp; // Flipped polarity to prevent runaway
    
    // Clamp effort to (-1, 1)
    if (effort > 1.0) effort = 1.0;
    if (effort < -1.0) effort = -1.0;

    // 4. Map Effort back to Servo Output (1500 +/- 1000us)
    outputUs = 1500 + (int32_t)(effort * 1000.0);

    // 5. Actuate
    if (systemEnabled && inputUs != 0) {
        servoOut.writeMicroseconds(outputUs);
    } else {
        servoOut.writeMicroseconds(1500);
    }

    // 6. Telemetry
    if (millis() - lastReport > 50) {
        char buf[128];
        snprintf(buf, sizeof(buf), 
                 "Tgt:%5.2f | Cur:%5.2f | Err:%5.2f | Out:%4lu | Gain:%0.2f | %s", 
                 targetNorm, currentNorm, error, outputUs, kp,
                 systemEnabled ? "RUN" : "STOP");
        Serial.println(buf);
        lastReport = millis();
    }
}
