# Wiring and Power

Wiring for the fixed sorting cell: a Raspberry Pi 4 running ROS 2, an Arduino
Uno driving a PCA9685 servo board over I2C, and a PN532 RFID reader on the
Pi's own I2C bus.

```
Raspberry Pi 4 ──USB serial──► Arduino Uno ──I2C──► PCA9685 ──► 6 servos
      ▲                                               ▲
      └──I2C── PN532                    separate servo supply (V+)
```

Items marked **confirm against your build** are not stated in the firmware or
code. Check them on your own cell.

---

## 1. Raspberry Pi to PN532 (I2C)

| PN532 pin | Raspberry Pi pin | Notes |
|---|---|---|
| 5V | Physical pin 1 (3.3V) | The PN532 board is fed from the Pi's **3.3V** pin |
| GND | Physical pin 6 (GND) | |
| MO/SDA/TX | Physical pin 3 (GPIO2, SDA) | |
| NSS/SCL/RX | Physical pin 5 (GPIO3, SCL) | |
| MI | not connected | |
| SCK | not connected | |
| RST | not connected | |
| IRQ | not connected | |

> **Do not connect the PN532 5V pin to the Pi's 5V pins (physical pins 2 or 4).**

### PN532 mode switches (I2C mode)

| Switch | Position |
|---|---|
| SET0 | H |
| SET1 | L |

- I2C bus on the Pi: `/dev/i2c-1` (enable with `dtparam=i2c_arm=on` in `/boot/firmware/config.txt`).
- I2C address: `0x24`. `i2cdetect -y 1` should show `24`.
- The address is the `rfid_reader` node's `i2c_address` parameter (default `0x24`), so a board at a different address does not need a code change.

---

## 2. Raspberry Pi to Arduino Uno (USB serial)

| Item | Value |
|---|---|
| Link | USB cable, Pi to Arduino |
| Device on the Pi | `/dev/ttyUSB0` on this cell (the code's default is `/dev/ttyACM0`; pass `serial_port:=/dev/ttyUSB0`) |
| Baud rate | 115200, 8N1 |
| Line ending | `\n` (a `\r` is ignored) |
| Boot delay | The Python side waits 2 s after opening the port, because opening it resets the Uno |

Only one program at a time can have the serial port open: the ROS
`arm_controller` node **or** a serial monitor, never both.

---

## 3. Arduino Uno to PCA9685 (I2C)

The Arduino's I2C pins go to the PCA9685's SDA and SCL, plus a shared ground.
The firmware uses the default `Wire` bus and the PCA9685's default address
`0x40`.

| Arduino Uno | PCA9685 | Notes |
|---|---|---|
| SDA (A4) | SDA | A4/A5 are the Uno's hardware I2C pins, as listed in the earlier subsystem docs. On an Uno R3 the SDA/SCL header pins are the same lines. **Confirm against your build.** |
| SCL (A5) | SCL | **Confirm against your build.** |
| GND | GND | Required. This is also the common ground with the servo supply. |
| 5V | VCC (logic) | PCA9685 logic power, as listed in the earlier subsystem docs. **Confirm against your build.** |

---

## 4. Servo channel map and calibration

Taken from `arduino/arm_controller/arm_controller.ino` (`CH_*` constants and
`SERVO_CALIBRATION`). Pulse values are PCA9685 12-bit counts at 50 Hz. The
firmware maps 0° to *min*, 90° to *mid* and 180° to *max*, piecewise linear
through *mid*.

| Channel | Joint | Servo | Serial command | Pulse min | Pulse mid | Pulse max |
|---|---|---|---|---|---|---|
| CH0 | Base | MG996R | `BASE:<angle>` | 100 | 310 | 520 |
| CH1 | Shoulder | MG996R | `SHOULDER:<angle>` | 100 | 310 | 520 |
| CH2 | Elbow | MG996R | `ELBOW:<angle>` | 100 | 310 | 520 |
| CH3 | Wrist roll | MG90S | `WRIST:<angle>` | 87 | 313 | 538 |
| CH4 | Wrist pitch | MG90S | `WRIST_PITCH:<angle>` | 87 | 313 | 538 |
| CH5 | Gripper | MG90S | `GRIPPER:OPEN` / `GRIPPER:CLOSE` | 87 | 311 | 535 |

Gripper angles in the firmware: **open = 150°, close = 20°**
(`GRIPPER_OPEN_ANGLE`, `GRIPPER_CLOSE_ANGLE`). Changing either one means
re-flashing the Arduino.

---

## 5. Power rules

| Rule | Why |
|---|---|
| Run the servos from a **separate supply** into the PCA9685 **V+ screw terminal** | Six servos under load draw far more current than the Arduino or USB can supply |
| **Share ground** between the servo supply, the PCA9685 and the Arduino | Without a common ground the PWM signals have no reference |
| **Never** power the servos from the Arduino's 5V pin | Brownouts, Arduino resets, possible damage to the Pi's USB port |
| Use a supply with **several amps of headroom** | A USB power bank caused shaking and stalling during testing |
| The Arduino is powered over USB from the Pi (logic only) | |

The firmware's header comment suggests a 5–6 V high-current supply (for
example a mains supply feeding a high-current buck converter). The exact
supply used on this cell, with its voltage and current rating:
**confirm against your build.**
