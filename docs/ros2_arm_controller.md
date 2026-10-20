# ROS 2 Arm Controller and Sorting Sequence

The ROS 2 side of the sorting cell (package `sorting_cell`, ROS 2 Jazzy on
Ubuntu 24.04, Raspberry Pi 4): the `arm_controller` node that talks to the
Arduino, and how `sorting_controller` drives it through a cycle. The
README has the full node, topic and parameter tables.

---

## 1. Structure

```
/arm/command (std_msgs/String)
      │
      ▼
arm_controller node                      sorting_cell/arm_controller.py
  ├── ArmCommandParser   syntax + joint-limit checks, ROS → wire format
  ├── ArmControllerCore  EXECUTING / OK / ERROR status flow
  └── SerialInterface    pyserial or mock, 2 s boot delay   serial_interface.py
      │
      ├──► /arm/status (std_msgs/String)
      ▼
USB serial 115200 8N1 '\n'  ──►  Arduino Uno (arm_controller.ino)
```

---

## 2. `/arm/command`

`std_msgs/msg/String`. The verb is case-insensitive.

| ROS command | Sent on the wire | Arduino reply |
|---|---|---|
| `BASE <angle>` | `BASE:<int>` | `OK:BASE` |
| `SHOULDER <angle>` | `SHOULDER:<int>` | `OK:SHOULDER` |
| `ELBOW <angle>` | `ELBOW:<int>` | `OK:ELBOW` |
| `WRIST <angle>` | `WRIST:<int>` | `OK:WRIST` (wrist roll, CH3) |
| `WRIST_PITCH <angle>` | `WRIST_PITCH:<int>` | `OK:WRIST_PITCH` (CH4) |
| `GRIPPER OPEN` | `GRIPPER:OPEN` | `OK:GRIPPER` |
| `GRIPPER CLOSE` | `GRIPPER:CLOSE` | `OK:GRIPPER` |
| `HOME` | `HOME` | `OK:HOME` (firmware all-zero pose; **not limit-checked; do not send to the assembled arm**) |

Angles may have decimals and are rounded to whole degrees before sending.

```bash
ros2 topic pub --once /arm/command std_msgs/msg/String "data: 'BASE 55'"
ros2 topic pub --once /arm/command std_msgs/msg/String "data: 'GRIPPER OPEN'"
```

### Joint limits

Checked before anything is sent. These are the `joint_limits.<joint>.min/max`
parameters (defaults below), mirrored by the `ArmCommandParser` defaults.

| Joint | Min | Max |
|---|---|---|
| base | 0 | 180 |
| shoulder | 15 | 165 |
| elbow | 15 | 175 |
| wrist | 0 | 180 |
| wrist_pitch | 0 | 180 |

The elbow maximum is 175 because `PICK_DOWN_POSITION` and
`RFID_SCAN_POSITION` use elbow = 175.

---

## 3. `/arm/status`

`std_msgs/msg/String`.

| Status | When |
|---|---|
| `IDLE` | Node started and serial (or mock) connected |
| `ERROR:SERIAL_CONNECTION` | Port could not be opened at startup |
| `EXECUTING:<action>` | A valid command is being sent (`BASE`, `SHOULDER`, `ELBOW`, `WRIST`, `WRIST_PITCH`, `GRIPPER`, `HOME`) |
| `OK:<action>` | Arduino accepted it |
| `ERROR:INVALID_COMMAND` | Failed ROS validation (syntax, unknown verb, outside the limits; the detail, e.g. `ANGLE_OUT_OF_LIMITS:...`, is in the node log), or the Arduino rejected it |
| `ERROR:SERIAL_TIMEOUT` | No reply line within `serial_timeout` (1 s) |
| `ERROR:ARDUINO_DISCONNECTED` | Write or read failed, or reconnecting failed |
| `ERROR:MALFORMED_RESPONSE:<text>` | The reply did not start with `OK` or `ERROR` |

`OK` means the Arduino **accepted** the target, not that the servo has
arrived.

### Parameters

| Parameter | Default |
|---|---|
| `mock_mode` | `false` |
| `serial_port` | `/dev/ttyACM0` (this cell: `/dev/ttyUSB0`) |
| `baud_rate` | `115200` |
| `serial_timeout` | `1.0` s |
| `joint_limits.*` | see above |
| `home_positions.*` | `90.0`, declared but unused |

The serial layer waits **2 s** after opening the port, because the Uno
resets when it connects, then clears the input buffer. Leftover reply bytes
are kept for the next read rather than discarded.

---

## 4. How `sorting_controller` drives the arm

`sorting_controller` publishes one `/arm/command` at a time and waits up to
`arm_command_timeout` (5 s) for an `OK:` or `ERROR:` on `/arm/status`.

A named pose becomes five commands in this fixed order: `BASE`, `SHOULDER`,
`ELBOW`, `WRIST`, `WRIST_PITCH`. Between them it waits `joint_gap_seconds`
(0.6 s), and after the last one `move_settle_seconds` (2 s). After each
gripper command it waits `gripper_settle_seconds` (1 s). At the scan pose it
waits a further `rfid_settle_seconds` (0.5 s) before waiting for a tag.

Cycle sequence:

| Step | `/sorting/status` | Arm action |
|---|---|---|
| 1 | `CYCLE_START` | — |
| 2 | `EXECUTING:MOVE_TO_HOME_POSITION` | `HOME_POSITION` |
| 3 | `EXECUTING:OPEN_GRIPPER` | `GRIPPER OPEN` |
| 4 | `EXECUTING:MOVE_TO_PICK_DOWN_POSITION` | `PICK_DOWN_POSITION` |
| 5 | `EXECUTING:GRIP` | `GRIPPER CLOSE` |
| 6 | `EXECUTING:LIFT_AFTER_GRIP` | `LIFT_POSITION` |
| 7 | `EXECUTING:MOVE_TO_RFID_SCAN` | `RFID_SCAN_POSITION` + 0.5 s |
| 8 | `WAITING_FOR_RFID` | waits up to `rfid_timeout` (15 s) |
| 9 | `RFID_DETECTED:<uid>`, `IDENTIFIED:<name>`, `BIN_SELECTED:<bin>` | — |
| 10 | `EXECUTING:MOVE_TO_FLOATING` | `FLOATING_POSITION` |
| 11 | `EXECUTING:MOVE_TO_BIN:<bin>` | `<bin>_POSITION` |
| 12 | `EXECUTING:RELEASE` | `GRIPPER OPEN` |
| 13 | `REPORT:SORTED:<uid>:<name>:<bin>` | `HOME_POSITION` |
| 14 | `IDLE` | — |

On `ERROR:RFID_TIMEOUT`, `ERROR:UNKNOWN_UID:<uid>`, `ERROR:ARM_FAILURE:...`
or `ERROR:INVALID_CONFIGURATION:MISSING_POSITION:...`, the controller tries
one move to `HOME_POSITION` **without opening the gripper**, then publishes
`IDLE`. If the final move home of a successful cycle fails, the error is
published and the controller stops there without `IDLE`.

### Current poses (`config/arm_positions.yaml`)

| Pose | base | shoulder | elbow | wrist | wrist_pitch |
|---|---|---|---|---|---|
| `HOME_POSITION` | 0 | 105 | 95 | 100 | 90 |
| `PICK_DOWN_POSITION` | 0 | 85 | 175 | 35 | 95 |
| `LIFT_POSITION` (derived, not measured; works on the arm in full cycles) | 0 | 85 | 125 | 35 | 95 |
| `RFID_SCAN_POSITION` | 55 | 90 | 175 | 0 | 100 |
| `FLOATING_POSITION` | 55 | 110 | 155 | 15 | 100 |
| `BIN_A_POSITION` | 85 | 110 | 155 | 15 | 100 |
| `BIN_B_POSITION` (shoulder/wrist aligned to BIN_A, checked on the arm) | 110 | 110 | 155 | 15 | 100 |

---

## 5. Mock mode

```bash
ros2 run sorting_cell arm_controller --ros-args -p mock_mode:=true
# or the whole cell:
ros2 launch sorting_cell sorting_cell.launch.xml mock_mode:=true
```

No port is opened, and `SerialInterface` answers with the replies the real
firmware would give:

```text
[INFO] [arm_controller]: Received command: BASE 90
[INFO] [arm_controller]: MOCK SERIAL -> BASE:90
[INFO] [arm_controller]: MOCK ARDUINO -> OK:BASE
```

---

## 6. Verification

| Item | Status |
|---|---|
| Serial link from the Pi to the Arduino | **Checked on hardware** |
| All joints and the gripper moving through ROS | **Checked on hardware** |
| PN532 reads through the `rfid_reader` node | **Checked on hardware** |
| Full pick, scan and sort cycles into both bins | **Checked on hardware** |
| Command parsing and joint-limit checks | Pending on hardware (unit tests only) |
| Serial error paths (timeout, disconnect, malformed reply) | Pending on hardware (unit tests only) |
| Recovery and failure paths of the cycle | Pending on hardware (unit tests only) |
| Settle and joint-gap waits | Pending on hardware (unit tests only) |
| Config loading and validation | Pending on hardware (unit tests only) |
| Mock mode through the launch file | Pending |

Unit tests: `cd ros2_ws/src/sorting_cell && python3 -m pytest` (234 tests,
Python logic only, no hardware).
