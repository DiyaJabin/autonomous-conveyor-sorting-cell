# Robotic Arm Subsystem: Arduino & PCA9685 Integration Guide

This document describes the low-level hardware control layer for the 6-DOF robotic arm subsystem of the **Autonomous Conveyor-Based Robotic Sorting Cell** (Review 2).

---

## 1. System Architecture

```
Ubuntu 22.04 Laptop (ROS 2 Humble)
       │
       │  Topic: /arm/command (std_msgs/msg/String)
       ▼
[arm_controller node]
       │
       │  USB Serial (115200 baud, newline-terminated ascii)
       ▼
[Arduino Uno]
       │
       │  I2C (SDA: A4, SCL: A5 @ 50 Hz PWM)
       ▼
[Adafruit PCA9685 16-Channel 12-bit PWM Driver]
       ├── CH0: BASE Servo (MG996R)
       ├── CH1: SHOULDER Servo (MG996R)
       ├── CH2: ELBOW Servo (MG996R)
       ├── CH3: WRIST Servo (MG996R/MG90S)
       ├── CH4: GRIPPER Servo (MG90S)
       └── CH5: EXTRA/SPARE Servo (MG90S)
```

---

## 2. Power Architecture & Electrical Safety

> [!CAUTION]
> **CRITICAL POWER WARNING: NEVER POWER SERVOS FROM ARDUINO 5V OR USB**
> - The 6 RC servos (3× high-torque MG996R + 3× MG90S) draw significant stall current (MG996R can draw > 2.5A each under load).
> - Attempting to power servos from the Arduino 5V pin or USB port will cause brownouts, MCU reboots, or permanently damage USB ports on the host computer.

### Safe Power Distribution
1. **Logic Power**: The Arduino Uno is powered via the USB connection to the host computer (supplies ATmega328P MCU and PCA9685 logic `VCC`).
2. **Servo Motor Power**: An external 12V DC power supply is stepped down via a high-current DC-DC buck converter (rated at 5V–6V, minimum 8A–10A continuous output) connected directly to the **PCA9685 V+ and GND screw terminals**.
3. **Common Ground**: A common ground reference **MUST** be connected between the Arduino GND and PCA9685 GND.

### Pinout Connections

| Arduino Uno Pin | PCA9685 Pin | Description |
|---|---|---|
| `5V` | `VCC` | Logic power supply (+5V) |
| `GND` | `GND` | Common ground reference |
| `A4` | `SDA` | I2C Data line |
| `A5` | `SCL` | I2C Clock line |
| *(External Buck 5V-6V)* | `V+ (Screw Terminal)` | Dedicated servo motor power |
| *(External Buck GND)* | `GND (Screw Terminal)` | Dedicated servo motor ground |

---

## 3. Serial Communication Protocol

- **Baud Rate**: `115200`
- **Data Format**: 8 data bits, no parity, 1 stop bit (8N1)
- **Line Ending**: Newline `\n` (CR `\r` is safely ignored)

### Command Set (ROS 2 → Arduino)

| Command | Example | Description |
|---|---|---|
| `HOME` | `HOME\n` | Moves all joints to configured software home positions |
| `BASE:<angle>` | `BASE:90\n` | Commands the base rotation joint to target angle |
| `SHOULDER:<angle>` | `SHOULDER:60\n` | Commands the shoulder joint to target angle |
| `ELBOW:<angle>` | `ELBOW:120\n` | Commands the elbow joint to target angle |
| `WRIST:<angle>` | `WRIST:90\n` | Commands the wrist joint to target angle |
| `GRIPPER:OPEN` | `GRIPPER:OPEN\n` | Sets gripper jaws to open angle |
| `GRIPPER:CLOSE` | `GRIPPER:CLOSE\n` | Sets gripper jaws to closed angle |

### Response Set (Arduino → ROS 2)

| Response | Meaning |
|---|---|
| `OK:HOME` | Homing sequence executed successfully |
| `OK:BASE` | Base angle command executed |
| `OK:SHOULDER` | Shoulder angle command executed |
| `OK:ELBOW` | Elbow angle command executed |
| `OK:WRIST` | Wrist angle command executed |
| `OK:GRIPPER` | Gripper state command executed |
| `ERROR:INVALID_COMMAND` | Malformed syntax, unknown target, or non-numeric value received |

---

## 4. PCA9685 Servo Channel Mapping

> [!IMPORTANT]
> The channel assignments below are **uncalibrated software placeholders**. The physical hardware team must verify actual wiring before enabling servo power.

| Channel | Joint Name | Servo Model | Placeholder Defaults | Calibrated Limits (Hardware Team) |
|---|---|---|---|---|
| `0` | Base | MG996R | `0° – 180°`, Home: `90°` | *TBD by Hardware Team* |
| `1` | Shoulder | MG996R | `15° – 165°`, Home: `90°` | *TBD by Hardware Team* |
| `2` | Elbow | MG996R | `15° – 175°`, Home: `90°` | *TBD by Hardware Team* |
| `3` | Wrist | MG90S / MG996R | `0° – 180°`, Home: `90°` | *TBD by Hardware Team* |
| `4` | Gripper | MG90S | Open: `30°`, Close: `90°` | *TBD by Hardware Team* |
| `5` | Spare / Aux | MG90S | Home: `90°` | *TBD by Hardware Team* |

---

## 5. Incremental Hardware Bring-Up Checklist

To prevent mechanical damage, do **NOT** power all 6 servos at once. Follow this gradual bring-up procedure:

1. **Step 1: Serial Interface Loopback Test (No Servos, No External Power)**
   - Connect Arduino Uno via USB to PC.
   - Open Arduino IDE Serial Monitor (115200 baud, newline `\n`).
   - Send `BASE:90` → Verify response: `OK:BASE`.
   - Send `GRIPPER:OPEN` → Verify response: `OK:GRIPPER`.
   - Send `BASE:INVALID` → Verify response: `ERROR:INVALID_COMMAND`.

2. **Step 2: Single Servo Test (1 Servo)**
   - Connect external 5V/6V supply to PCA9685 screw terminals.
   - Plug **1 servo only** into Channel 0 (Base).
   - Send `BASE:90` → Confirm smooth servo motion without jitter or brownout.

3. **Step 3: Two Servos Test (2 Servos)**
   - Add second servo on Channel 1 (Shoulder).
   - Test commanding Base and Shoulder independently.

4. **Step 4: Three Servos Test (3 Servos)**
   - Add third servo on Channel 2 (Elbow).
   - Check current draw on power supply when moving arm joints.

5. **Step 5: Full 6-Servo Arm Assembly**
   - Connect remaining wrist and gripper servos (Channels 3, 4, 5).
   - Record true mechanical minimum, maximum, and home angles for calibration.
   - Update ROS 2 launch parameters with verified physical limits.
