/*
 * =====================================================================================
 * Autonomous Conveyor-Based Robotic Sorting Cell
 * Subsystem: Robotic Arm Low-Level Hardware Controller
 * Target: Arduino Uno + Adafruit PCA9685 16-Channel 12-Bit PWM Driver
 *
 * RESPONSIBILITIES:
 * 1. Receive newline-terminated serial commands from ROS 2 arm_controller (or Serial Monitor)
 * 2. Parse and validate commands
 * 3. Communicate with PCA9685 over I2C
 * 4. Drive 6 servos (3x MG996R high torque + 3x MG90S lightweight)
 * 5. Return synchronous OK:<ACTION> / ERROR:INVALID_COMMAND responses
 *
 * =====================================================================================
 * CRITICAL ELECTRICAL & HARDWARE SAFETY REQUIREMENTS:
 * =====================================================================================
 * 1. SEPARATE SERVO POWER:
 *    - NEVER power physical servos directly from the Arduino 5V rail or USB!
 *    - 6 servos under mechanical load will draw up to 5-10A peak, causing Arduino
 *      brownout resets or USB port damage.
 *    - Servos must be powered by an external 5V-6V high-current DC power supply
 *      (e.g., 12V DC mains supply -> high-current buck converter -> PCA9685 V+ screw terminals).
 *    - Arduino USB supplies communication and logic power ONLY.
 *    - Arduino GND MUST be connected to PCA9685 GND (Common Ground).
 *
 * 2. SERVO CALIBRATION & MECHANICAL SAFETY:
 *    - Channel mappings and pulse ranges below are UNCALIBRATED SOFTWARE PLACEHOLDERS.
 *    - The hardware team must calibrate actual mechanical limits and PCA9685 channel mappings
 *      before commanding physical servos.
 *    - Do not assume 0 or 180 degrees are mechanically safe.
 *    - Test physical servos incrementally: 1 servo -> 2 servos -> 3 servos -> 6 servos.
 * =====================================================================================
 */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

// Initialize PCA9685 instance at default I2C address (0x40)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// =====================================================================================
// CONFIGURATION & CALIBRATION (Centralized Constants)
// =====================================================================================

// Serial Baud Rate (must match ROS 2 arm_controller serial_interface baud_rate)
const unsigned long SERIAL_BAUD_RATE = 115200;

// Servo PWM frequency (50 Hz = standard 20ms period for analog & digital RC servos)
const float SERVO_PWM_FREQ = 50.0;

// PCA9685 Channel Assignments
const uint8_t CH_BASE        = 0;
const uint8_t CH_SHOULDER    = 1;
const uint8_t CH_ELBOW       = 2;
const uint8_t CH_WRIST_ROLL  = 3;
const uint8_t CH_WRIST_PITCH = 4;
const uint8_t CH_CLAW        = 5;

// PCA9685 12-bit PWM pulse width boundaries (4096 counts = 20ms period at 50Hz).
// Per-channel calibration measured by the hardware team for each physical
// servo (MIN/MID/MAX pulse counts). Array index == PCA9685 channel number.
struct ServoCalibration {
    uint16_t pulseMin;
    uint16_t pulseMid;
    uint16_t pulseMax;
};

const ServoCalibration SERVO_CALIBRATION[6] = {
    /* CH0 Base        MG996R */ {100, 310, 520},
    /* CH1 Shoulder    MG996R */ {100, 310, 520},
    /* CH2 Elbow       MG996R */ {100, 310, 520},
    /* CH3 Wrist Roll  MG90S  */ { 87, 313, 538},
    /* CH4 Wrist Pitch MG90S  */ { 87, 313, 538},
    /* CH5 Gripper     MG90S  */ { 87, 311, 535},
};

// Home Position Angles
const float HOME_ANGLE_BASE        = 0.0;
const float HOME_ANGLE_SHOULDER    = 0.0;
const float HOME_ANGLE_ELBOW       = 0.0;
const float HOME_ANGLE_WRIST_ROLL  = 0.0;
const float HOME_ANGLE_WRIST_PITCH = 0.0;

// Gripper Angular Setpoints
const float GRIPPER_OPEN_ANGLE  = 150.0;
const float GRIPPER_CLOSE_ANGLE = 20.0;

// Serial buffer configuration
const size_t BUFFER_SIZE = 64;
char serialBuffer[BUFFER_SIZE];
size_t bufferIndex = 0;

// =====================================================================================
// PWM CONVERSION & SERVO CONTROL HELPER FUNCTIONS
// =====================================================================================

bool isAngleWithinLimits(float angle, float minAngle, float maxAngle)
{
    return angle >= minAngle && angle <= maxAngle;
}

uint16_t angleToPwmCount(
    float angle,
    float minAngle,
    float maxAngle,
    const ServoCalibration &cal
) {
    // Piecewise-linear through the calibrated MID point (not just MIN/MAX),
    // so per-servo calibration data is honored even if a joint's true pulse
    // response is not perfectly symmetric between its two halves.
    float midAngle = (minAngle + maxAngle) / 2.0;

    if (angle <= midAngle) {
        float ratio = (angle - minAngle) / (midAngle - minAngle);
        return (uint16_t)(cal.pulseMin + ratio * (cal.pulseMid - cal.pulseMin) + 0.5);
    } else {
        float ratio = (angle - midAngle) / (maxAngle - midAngle);
        return (uint16_t)(cal.pulseMid + ratio * (cal.pulseMax - cal.pulseMid) + 0.5);
    }
}

bool setServoAngle(
    uint8_t channel,
    float angle,
    float minAngle = 0.0,
    float maxAngle = 180.0
) {
    // Reject invalid angles instead of silently clamping them.
    if (angle < minAngle || angle > maxAngle) {
        return false;
    }

    uint16_t pwmVal = angleToPwmCount(
        angle,
        minAngle,
        maxAngle,
        SERVO_CALIBRATION[channel]
    );

    pwm.setPWM(channel, 0, pwmVal);
    return true;
}

// Move all joints to their configured HOME placeholder positions.
void executeHome() {
    setServoAngle(CH_BASE, HOME_ANGLE_BASE);
    setServoAngle(CH_SHOULDER, HOME_ANGLE_SHOULDER);
    setServoAngle(CH_ELBOW, HOME_ANGLE_ELBOW);
    setServoAngle(CH_WRIST_ROLL, HOME_ANGLE_WRIST_ROLL);
    setServoAngle(CH_WRIST_PITCH, HOME_ANGLE_WRIST_PITCH);
    setServoAngle(CH_CLAW, GRIPPER_OPEN_ANGLE);
}

// =====================================================================================
// COMMAND PARSER & DISPATCHER
// =====================================================================================

void processCommand(char* cmd) {

    // Trim leading whitespace
    while (*cmd == ' ' || *cmd == '\t') {
        cmd++;
    }

    // Trim trailing whitespace
    int len = strlen(cmd);
    while (len > 0 &&
           (cmd[len - 1] == ' ' ||
            cmd[len - 1] == '\t' ||
            cmd[len - 1] == '\r')) {
        cmd[--len] = '\0';
    }

    if (len == 0) {
        Serial.println(F("ERROR:INVALID_COMMAND"));
        return;
    }

    // HOME command
    if (strcmp(cmd, "HOME") == 0) {
        executeHome();
        Serial.println(F("OK:HOME"));
        return;
    }

    // Commands with colon separator: <TARGET>:<VALUE>
    char* colonPos = strchr(cmd, ':');

    if (colonPos == NULL) {
        Serial.println(F("ERROR:INVALID_COMMAND"));
        return;
    }

    *colonPos = '\0';

    char* target = cmd;
    char* valueStr = colonPos + 1;

    // Validate that value string is not empty
    if (*valueStr == '\0') {
        Serial.println(F("ERROR:INVALID_COMMAND"));
        return;
    }

    // GRIPPER command
    if (strcmp(target, "GRIPPER") == 0) {

        if (strcmp(valueStr, "OPEN") == 0) {
            setServoAngle(CH_CLAW, GRIPPER_OPEN_ANGLE);
            Serial.println(F("OK:GRIPPER"));
            return;

        } else if (strcmp(valueStr, "CLOSE") == 0) {
            setServoAngle(CH_CLAW, GRIPPER_CLOSE_ANGLE);
            Serial.println(F("OK:GRIPPER"));
            return;

        } else {
            Serial.println(F("ERROR:INVALID_COMMAND"));
            return;
        }
    }

    // Joint angle commands
    char* endPtr = NULL;
    float angle = strtod(valueStr, &endPtr);

    // If conversion failed
    if (endPtr == valueStr) {
        Serial.println(F("ERROR:INVALID_COMMAND"));
        return;
    }

    // Ensure rest of valueStr is only whitespace
    while (*endPtr != '\0') {
        if (*endPtr != ' ' && *endPtr != '\t') {
            Serial.println(F("ERROR:INVALID_COMMAND"));
            return;
        }
        endPtr++;
    }

    const float MIN_ANGLE = 0.0;
    const float MAX_ANGLE = 180.0;

    if (angle < MIN_ANGLE || angle > MAX_ANGLE) {
        Serial.println(F("ERROR:INVALID_COMMAND"));
        return;
    }

    if (strcmp(target, "BASE") == 0) {

        if (setServoAngle(CH_BASE, angle, MIN_ANGLE, MAX_ANGLE)) {
            Serial.println(F("OK:BASE"));
        } else {
            Serial.println(F("ERROR:INVALID_COMMAND"));
        }

    } else if (strcmp(target, "SHOULDER") == 0) {

        if (setServoAngle(CH_SHOULDER, angle, MIN_ANGLE, MAX_ANGLE)) {
            Serial.println(F("OK:SHOULDER"));
        } else {
            Serial.println(F("ERROR:INVALID_COMMAND"));
        }

    } else if (strcmp(target, "ELBOW") == 0) {

        if (setServoAngle(CH_ELBOW, angle, MIN_ANGLE, MAX_ANGLE)) {
            Serial.println(F("OK:ELBOW"));
        } else {
            Serial.println(F("ERROR:INVALID_COMMAND"));
        }

    } else if (strcmp(target, "WRIST") == 0) {

        if (setServoAngle(CH_WRIST_ROLL, angle, MIN_ANGLE, MAX_ANGLE)) {
            Serial.println(F("OK:WRIST"));
        } else {
            Serial.println(F("ERROR:INVALID_COMMAND"));
        }

    } else if (strcmp(target, "WRIST_PITCH") == 0) {

        if (setServoAngle(CH_WRIST_PITCH, angle, MIN_ANGLE, MAX_ANGLE)) {
            Serial.println(F("OK:WRIST_PITCH"));
        } else {
            Serial.println(F("ERROR:INVALID_COMMAND"));
        }

    } else {
        Serial.println(F("ERROR:INVALID_COMMAND"));
    }
}

// =====================================================================================
// ARDUINO MAIN SETUP & LOOP
// =====================================================================================

void setup() {

    // Initialize USB Serial communication
    Serial.begin(SERIAL_BAUD_RATE);

    while (!Serial) {
        ;
    }

    // Initialize I2C communication and PCA9685 PWM driver
    Wire.begin();
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_PWM_FREQ);

    // Safety pause after initialization
    delay(10);

    // Servos are deliberately NOT homed on powerup.
}

void loop() {

    // Non-blocking serial reception
    while (Serial.available() > 0) {

        char inChar = (char)Serial.read();

        // End of line received
        if (inChar == '\n') {

            serialBuffer[bufferIndex] = '\0';
            processCommand(serialBuffer);
            bufferIndex = 0;
        }

        // Discard carriage returns
        else if (inChar == '\r') {
            continue;
        }

        // Append character with overflow protection
        else {

            if (bufferIndex < BUFFER_SIZE - 1) {
                serialBuffer[bufferIndex++] = inChar;

            } else {

                bufferIndex = 0;
                Serial.println(F("ERROR:INVALID_COMMAND"));
            }
        }
    }
}