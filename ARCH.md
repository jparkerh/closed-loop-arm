# Closed-Loop Robotic Arm Architecture

```mermaid
graph TD
    subgraph "External Hardware"
        ENC_HW[PWM Encoder GP29]
        CMD_HW[Servo Input GP2]
        OVR_HW[Override GP3]
        MAN_HW[Manual GP5]
        LOG_HW[Datalogger GP0/GP1]
        MOT_HW[Motor/ESC GP4]
    end

    subgraph "Core 0 (Fast Path - 244Hz Sync)"
        direction TB
        E_ISR[Encoder ISR]
        PID[PID Controller]
        ACT[Servo Library Write]
        WD0[Heartbeat 0]
        
        ENC_HW --> E_ISR
        E_ISR --> PID
        PID --> ACT
        ACT --> MOT_HW
    end

    subgraph "Shared Memory (Atomic Floats)"
        S_TGT((shared_targetNorm))
        S_CUR((shared_currentNorm))
        S_OUT((shared_finalOutputUs))
        S_AGE((shared_encoderAge))
    end

    subgraph "Core 1 (Interface & Supervisor)"
        direction TB
        S_MEAS[Servo Input Measure]
        SER[Serial Command Parser]
        TEL[Telemetry Streamer (Serial1)]
        HWDG[Hardware Watchdog Feed]
        WD1[Heartbeat 1]
        
        CMD_HW --> S_MEAS
        SER --> TEL
    end

    %% Data Flow
    S_MEAS -.-> S_TGT
    S_TGT -.-> PID
    E_ISR -.-> S_CUR
    S_CUR -.-> TEL
    ACT -.-> S_OUT
    S_OUT -.-> TEL
    WD0 -.-> HWDG
    WD1 -.-> HWDG
    HWDG -- "watchdog_update()" --> RPI_WDG[RP2040 Silicon Watchdog]
```

## System Overview
A high-performance, dual-core control system for a robotic arm joint using an RP2040-Zero. The system translates standard RC Servo PWM signals into precise angular positions using high-resolution encoder feedback.

## Core Responsibilities

### Core 0 (The Master Controller)
*   **Encoder Processing:** Direct interface with the 244Hz PWM encoder on GP29.
*   **Synchronized Control Loop:** Executes the PID math exactly when a new encoder pulse arrives (minimizing phase lag).
*   **Actuation:** Uses the `Servo` library to update the motor (GP4) immediately after PID calculation.
*   **Heartbeat:** Maintains `heartbeat0` for the hardware watchdog.

### Core 1 (The Gatekeeper & Interface)
*   **Servo Input Measurement:** Measures the incoming 50Hz control pulse (1000-2000µs) on GP2.
*   **Hardware Watchdog Management:** Feeds the physical RP2040 watchdog timer (200ms) ONLY if both cores are reporting active heartbeats.
*   **Command Parsing:** Interprets serial tuning commands (P, I, D, R, S).
*   **Telemetry:** Formats and streams system state to Serial (USB) and Serial1 (UART) every 50ms.
*   **Safety Gate:** Monitors Core 0 health and provides a secondary shutdown of the motor if the master core fails.

## Control Logic
*   **Normalization:** All internal logic operates on a normalized `(-1.0, 1.0)` scale.
*   **Feedback Filtering:** One-Pole Low-Pass Filter ($y = \alpha \cdot x + (1-\alpha) \cdot y_{prev}$) on the encoder input.
*   **PID Controller:** Full Proportional-Integral-Derivative with Anti-Windup and Derivative damping.
*   **Sync:** Core 0 math is event-triggered by the 244Hz encoder, not a fixed timer.

## Safety Features
1.  **Hardware Watchdog:** 200ms full-chip reset if either core hangs.
2.  **Encoder Loss Watchdog:** Motor neutralized if feedback signal is lost for >100ms.
3.  **Command Loss Watchdog:** Target set to neutral if stick signal is lost for >200ms.
4.  **Startup Stabilization:** 5-second "WARMUP" period to allow encoder stabilization before zeroing and enabling movement.

## Development History (Phases)
1.  **Phase 1:** PlatformIO initialization & raw encoder pulse measurement.
2.  **Phase 2:** Continuous angle unwrapping logic & noise rejection (Velocity Cap).
3.  **Phase 3:** Integration of Servo Input & Basic Proportional control.
4.  **Phase 4:** Dual-core transition (Math on Core 0, I/O on Core 1).
5.  **Phase 5:** Hardware PWM migration & 1000Hz fixed-rate testing.
6.  **Phase 6:** Atomic synchronization (`float` transition) & Event-driven (Sync) math.
7.  **Phase 7:** Final stabilization & Hardware Watchdog integration.

## Pin Mapping (RP2040-Zero)
| Pin | Function | Direction |
|---|---|---|
| GP29 | Encoder Feedback | Input (PWM) |
| GP2 | Stick Command | Input (Servo) |
| GP3 | Override Switch | Input (Servo) |
| GP5 | Manual Control | Input (Servo) |
| GP4 | Motor Driver | Output (Servo PWM) |
| GP0 | Datalog TX (Serial1) | Output (UART) |
| GP1 | Datalog RX (Serial1) | Input (UART) |
