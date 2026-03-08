# Engineering Standards

## Motor Actuation
*   **Library:** The `Servo` library is MANDATORY for all motor/ESC PWM output. Do not use direct hardware PWM register manipulation for actuation, as the `Servo` library has proven most reliable for the target hardware.
*   **Ownership:** All motor actuation calls must reside exclusively on **Core 0** to prevent hardware resource contention.
