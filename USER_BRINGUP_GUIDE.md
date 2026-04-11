# User Bringup Guide: Closed-Loop Arm Controller

This guide summarizes the input signals and configuration options for the closed-loop arm controller board.

## 1. System Signal Overview

| Signal Name | Pin | Type | Function |
| :--- | :--- | :--- | :--- |
| **Servo Input** | GP2 | PWM | **Target:** The primary setpoint for the arm's position. |
| **Manual Input** | GP5 | PWM | **Direct Control:** Pass-through signal used in Manual mode. |
| **Override Switch**| GP3 | PWM | **Mode Toggle:** Switches between Autonomous and Manual control. |
| **Encoder** | GP29| Digital | **Feedback:** High-speed data on the arm's actual position. |
| **Command/Config**| UART | Serial | **Tuning:** Interface for PID gains and system enabling. |

---

## 2. Override Switch Functionality (GP3)

The **Override** signal is a binary mode selector. For documentation and user configuration, it follows the standard PWM range:

### **Closed-Loop Mode (1000µs / "Low")**
*   **Effect:** The system is under autonomous PID control.
*   **Logic:** The board compares the *Servo Input* (target) against the *Encoder* (actual) and calculates the motor output required to reach the target.

### **Manual Pass-Through (2000µs / "High")**
*   **Effect:** The internal PID logic is bypassed entirely.
*   **Logic:** The signal from the *Manual Input* is bridged directly to the output. This allows the operator to take physical control of the arm, serving as a safety "kill-switch" for the autonomous logic.

---

## 3. Input Options & Output Impact

| Input State (Override) | Active Control Source | Resulting Output (GP4) |
| :--- | :--- | :--- |
| **1000µs (Low)** | Internal PID Logic | Regulated PID effort (Closed-loop) |
| **2000µs (High)** | Manual Input (GP5) | Direct 1:1 Signal (Open-loop) |
| **No Signal / Error** | Failsafe Logic | Neutral (1500µs) |

---

## 4. Hardware Configuration & Wiring

- **Serial Telemetry:** 115200 baud (GP0=TX, GP1=RX).
- **Servo Library:** All actuation is handled via the standard `Servo` library on Core 0.
- **Power:** Ensure a common ground between the signal source (e.g., RC Receiver) and the board.

---

## 5. Safe Bringup Process

Follow these steps strictly for the first power-on to prevent mechanical damage or unexpected movement.

### **Phase A: Preparation**
1.  **Disconnect Linkages:** Drop all chains or mechanical linkages from the motor to the arm. The motor should be able to spin freely.
2.  **Transmitter Setup:** Ensure your transmitter is on and the **Override Switch** is set to **Manual Mode (2000µs / High)**.

### **Phase B: Power-On Sequence**
1.  **ESC Power:** Connect the ESC 5V line (BEC) to the board's power rail.
2.  **USB Warning:** **Do NOT connect the USB cable** during this initial phase. This prevents the logic board from powering up before the battery/ESC system is ready. The USB connection is only intended for telemetry and PID tuning once the hardware power sequence is verified.
3.  **Zeroing Note:** The system defines "Zero" as the position of the encoder at the moment of power-on (after a 5-second warmup).

### **Phase C: Manual Verification**
1.  **Test Output:** Use the **Manual Input** (stick) to verify the motor spins in both directions.
2.  **Set Offset:** Using the manual stick, move the motor a small amount (e.g., 30 degrees) away from its power-on zero position.

### **Phase D: Closed-Loop Verification**
1.  **Engage Logic:** Flip the **Override Switch** to **Closed-Loop Mode (1000µs / Low)**.
2.  **Return to Zero:** The motor should automatically move back to the power-on zero position.
3.  **Command Test:** Use the **Servo Input** stick. Verify that increasing the signal moves the motor in the expected direction.
4.  **Direction Check:** If the motor "runs away" (accelerates to max speed), the encoder or motor polarity is reversed. Power down immediately.

### **Phase E: Final Assembly**
1.  Once directional correctness is confirmed, power down and reconnect the chains/linkages.
2.  Perform a low-load test before full operation.

