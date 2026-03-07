#include <Arduino.h>
#include "Encoder.h"

// Instantiate the encoder on GP2
Encoder encoder(2);

void setup() {
    Serial.begin(115200);
    unsigned long start_wait = millis();
    while (!Serial && millis() - start_wait < 3000); 
    
    Serial.println("\n--- Encapsulated Encoder Monitor ---");
    encoder.begin();
}

unsigned long last_report = 0;

void loop() {
    // Check for new encoder data
    if (encoder.update()) {
        // Only report every 50ms to the serial port
        if (millis() - last_report > 50) {
            char buf[64];
            // Format: Raw padded to 7 chars (e.g. 359.1), Continuous padded to 10 chars (e.g. -12345.67)
            snprintf(buf, sizeof(buf), "Raw: %7.1f | Continuous: %10.2f", 
                     encoder.getRawAngle(), encoder.getContinuousAngle());
            Serial.println(buf);
            last_report = millis();
        }
    }
}
