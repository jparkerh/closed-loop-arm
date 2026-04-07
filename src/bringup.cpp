#include <Arduino.h>
#include "Encoder.h"

// Encoder on GP29
Encoder encoder(29, 0.5f);

void setup() {
    Serial.begin(115200);
    delay(2000);
    Serial.println("\n--- Encoder Bringup Utility ---");
    encoder.begin();
}

void loop() {
    if (encoder.update()) {
        static unsigned long lastPrint = 0;
        if (millis() - lastPrint > 100) {
            char buf[128];
            // Constant width formatting:
            // Raw: 000.0 to 359.9 (6 chars)
            // Continuous: -00000.00 (10 chars)
            // Period: 0000 (5 chars)
            snprintf(buf, sizeof(buf), 
                     "Raw: %6.1f | Continuous: %10.2f | Period: %5lu us", 
                     encoder.getRawAngle(), 
                     encoder.getContinuousAngle(), 
                     encoder.getPeriod());
            Serial.println(buf);
            lastPrint = millis();
        }
    }
}
