# Engineering Standards

## Motor Actuation
*   **Library:** The `RP2040_PWM` library is MANDATORY for all motor/ESC PWM output. This provides high-frequency (400Hz+) hardware-timed pulses that eliminate the jitter and "bounciness" associated with the standard 50Hz `Servo` library.
*   **Ownership:** All motor actuation calls must reside exclusively on **Core 0** to prevent hardware resource contention.
