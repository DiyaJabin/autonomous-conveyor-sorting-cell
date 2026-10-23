# Arduino Uno and PCA9685 Servo Subsystem

The low-level servo layer of the sorting cell: `arduino/arm_controller/arm_controller.ino`
on an Arduino Uno, driving six servos through a PCA9685 board. The Arduino
receives text commands over USB serial from the ROS 2 `arm_controller` node
on the Raspberry Pi (or from a serial monitor) and replies once per command.

See [../hardware/WIRING.md](../hardware/WIRING.md) for the full wiring and
power tables and [ros2_arm_controller.md](ros2_arm_controller.md) for the
ROS side.

---

## 1. Architecture

```
Raspberry Pi 4 (Ubuntu 24.04, ROS 2 Jazzy)
       │  arm_controller node
       │
       │  USB serial, 115200 baud 8N1, '\n'-terminated ASCII
       ▼
Arduino Uno  (arm_controller.ino)
       │
       │  I2C, PCA9685 at 0x40, 50 Hz PWM
       ▼
PCA9685 16-channel 12-bit PWM driver  ◄── separate servo supply on V+
       ├── CH0  Base         MG996R
       ├── CH1  Shoulder     MG996R
       ├── CH2  Elbow        MG996R
       ├── CH3  Wrist roll   MG90S
       ├── CH4  Wrist pitch  MG90S
       └── CH5  Gripper      MG90S
```

---

## 2. Power and wiring

> **Never power the servos from the Arduino's 5V pin or from USB.**

- The servos run from a **separate supply** into the PCA9685 **V+** screw terminal.
- That supply's ground is **shared** with the PCA9685 and the Arduino.
- Use a supply with **several amps of headroom**. A USB power bank caused shaking and stalling during testing.
- The Arduino is powered and talks over USB from the Pi.

| Arduino Uno | PCA9685 | Notes |
|---|---|---|
| SDA (A4) | SDA | Uno hardware I2C. **Confirm against your build.** |
| SCL (A5) | SCL | **Confirm against your build.** |
| GND | GND | Required (common ground) |
| 5V | VCC (logic) | Logic power only. **Confirm against your build.** |
| | V+ / GND screw terminal | Separate servo supply |

---

## 3. Channel map and calibration

From the `CH_*` constants and `SERVO_CALIBRATION` in the firmware. Pulses
are PCA9685 counts (4096 per 20 ms period at 50 Hz).

| Channel | Joint | Servo | Command | Min (0°) | Mid (90°) | Max (180°) |
|---|---|---|---|---|---|---|
| CH0 | Base | MG996R | `BASE` | 100 | 310 | 520 |
| CH1 | Shoulder | MG996R | `SHOULDER` | 100 | 310 | 520 |
| CH2 | Elbow | MG996R | `ELBOW` | 100 | 310 | 520 |
| CH3 | Wrist roll | MG90S | `WRIST` | 87 | 313 | 538 |
| CH4 | Wrist pitch | MG90S | `WRIST_PITCH` | 87 | 313 | 538 |
| CH5 | Gripper | MG90S | `GRIPPER` | 87 | 311 | 535 |

`angleToPwmCount()` interpolates linearly from 0° to 90° (min to mid) and
from 90° to 180° (mid to max).

| Constant | Value |
|---|---|
| `GRIPPER_OPEN_ANGLE` | 150° |
| `GRIPPER_CLOSE_ANGLE` | 20° |
| `HOME_ANGLE_*` (all five joints) | 0° |
| PWM frequency | 50 Hz (oscillator set to 27 MHz) |

---

## 4. Serial protocol

- 115200 baud, 8N1.
- One command per line, terminated by `\n`. A `\r` is discarded, and leading and trailing spaces and tabs are trimmed.
- The buffer holds 63 characters. A longer line is answered with `ERROR:INVALID_COMMAND`.
- The reply is sent as soon as the servo target is set, **before the servo has physically moved**. The ROS side adds settle waits for this reason.

| Command | Reply | Effect |
|---|---|---|
| `BASE:<angle>` | `OK:BASE` | CH0 |
| `SHOULDER:<angle>` | `OK:SHOULDER` | CH1 |
| `ELBOW:<angle>` | `OK:ELBOW` | CH2 |
| `WRIST:<angle>` | `OK:WRIST` | CH3, wrist roll |
| `WRIST_PITCH:<angle>` | `OK:WRIST_PITCH` | CH4, wrist pitch |
| `GRIPPER:OPEN` | `OK:GRIPPER` | CH5 to 150° |
| `GRIPPER:CLOSE` | `OK:GRIPPER` | CH5 to 20° |
| `HOME` | `OK:HOME` | All five joints to 0° and the gripper open |

`<angle>` is any number from 0 to 180 (parsed with `strtod`).

The only error reply is `ERROR:INVALID_COMMAND`. It is returned for an empty
line, a missing `:` or value, an unknown target, a gripper value other than
`OPEN`/`CLOSE`, a non-numeric angle or trailing characters, an angle outside
0–180, and a line that overflows the buffer.

The firmware applies **no per-joint limits** beyond 0–180. The joint limits
(for example elbow 15–175) are enforced by the ROS `arm_controller` node.

### Firmware HOME vs ROS HOME_POSITION

The firmware `HOME` command drives every joint to 0°, which is outside the
ROS shoulder and elbow limits (minimum 15). The sorting cycle never sends it.
Instead it moves to the measured `HOME_POSITION` pose from
`config/arm_positions.yaml`, one joint at a time. **Do not send `HOME` to the
assembled arm.**

### Boot behaviour

- `setup()` starts serial, then I2C and the PCA9685, then waits 10 ms. **Servos are not moved on power-up or reset.**
- Opening the serial port resets the Uno (DTR). The Python `SerialInterface` therefore waits **2 s** (`boot_delay`) after opening the port, then clears the input buffer before sending the first command.

---

## 5. Flashing

From the repository root, on the machine the Arduino is plugged into (stop
any ROS launch or serial monitor first):

```bash
arduino-cli core update-index
arduino-cli core install arduino:avr
arduino-cli lib install "Adafruit PWM Servo Driver Library"
arduino-cli compile --fqbn arduino:avr:uno arduino/arm_controller
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno arduino/arm_controller
```

Any change to the calibration, gripper angles or protocol needs a re-flash.

---

## 6. Manual jogging and re-measuring poses

1. Stop the ROS launch, since only one program can hold the port.
2. Open a serial monitor at 115200 baud with newline line endings.
3. Send one joint at a time in small steps, e.g. `BASE:55`, then `SHOULDER:90`. Expect `OK:<JOINT>`.
4. Never send `HOME`.
5. Record base, shoulder, elbow, wrist and wrist pitch, keep them inside the ROS joint limits, and enter them in `config/arm_positions.yaml`. Rebuild the package afterwards.

Bring-up order for a fresh or rebuilt arm: start with one servo, then two,
then three, then all six, checking the supply each time.

---

## 7. Verification

| Item | Status |
|---|---|
| Serial link from the Pi to the Arduino | **Checked on hardware** |
| All joints and the gripper moving through ROS | **Checked on hardware** |
| Full pick, scan and sort cycles into both bins | **Checked on hardware** |
| Firmware `HOME` on the assembled arm | Pending (deliberately not used) |
| Per-joint speed smoothing | Not implemented: servos are set straight to their target and move at full speed |
