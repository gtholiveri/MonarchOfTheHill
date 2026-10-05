#include "DriveTrain.h"

#include "config.h"

static int clampSpeed(int v) { return v < -255 ? -255 : (v > 255 ? 255 : v); }

// ===========================================================================
// ACEBOTT ESP32 Car Shield V2.0
//
// The shield does NOT expose the H-bridge to the ESP32. A co-processor on the
// shield does the switching, and the ESP32 talks to it over a 9600-baud serial
// link on GPIO16/17 (the ESP32's default Serial2 pins). One command is a
// 5-byte packet:
//
//     byte 0   motor index   1-4
//     byte 1   direction     1 = forward, 2 = reverse
//     byte 2   speed         0-100
//     byte 3   '\r'
//     byte 4   '\n'
//
// Quirks inherited from ACEBOTT's library, kept deliberately:
//   - Motors 1/2 and 3/4 have MIRRORED direction conventions, because they are
//     mounted facing opposite ways on the two sides. Motors 1,2 are the left
//     side and 3,4 the right.
//   - map() lands on 40 at an input of ~102, which the protocol treats as stop.
// ===========================================================================
#if DRIVE_BACKEND == DRIVE_ACEBOTT_SHIELD

static const int SHIELD_RX_PIN = 16;
static const int SHIELD_TX_PIN = 17;
static const long SHIELD_BAUD = 9600;

static int lastSent[5] = {0, 0, 0, 0, 0};   // index 1..4; the link is slow, so skip repeats

static void shieldMotor(uint8_t motor, int speed) {
    speed = clampSpeed(speed);
    if (speed == lastSent[motor]) return;
    lastSent[motor] = speed;

    uint8_t dir = (motor <= 2) ? (speed > 0 ? 1 : 2)
                               : (speed > 0 ? 2 : 1);

    uint8_t value = map(abs(speed), 0, 255, 0, 100);
    if (value == 40) value = 0;     // protocol quirk, see above

    Serial2.write(motor);
    Serial2.write(dir);
    Serial2.write(value);
    Serial2.write('\r');
    Serial2.write('\n');
}

bool DriveTrain::begin() {
    Serial2.begin(SHIELD_BAUD, SERIAL_8N1, SHIELD_RX_PIN, SHIELD_TX_PIN);
    delay(200);
    for (uint8_t m = 1; m <= 4; m++) { lastSent[m] = -999; shieldMotor(m, 0); }
    return true;
}

void DriveTrain::drive(int left, int right) {
    _left = clampSpeed(left);
    _right = clampSpeed(right);
    shieldMotor(1, _left);
    shieldMotor(2, _left);
    shieldMotor(3, _right);
    shieldMotor(4, _right);
}

// ===========================================================================
// SparkFun Motor Driver - Dual TB6612FNG
//
// Per motor: PWM pin sets speed, IN1/IN2 set direction.
//   IN1 H, IN2 L -> one way      IN1 L, IN2 H -> the other way
//   IN1 H, IN2 H -> short brake  (used for speed 0: resists being pushed)
// STBY must be HIGH or the chip ignores everything.
// ===========================================================================
#elif DRIVE_BACKEND == DRIVE_TB6612FNG

static const size_t N_LEFT = sizeof(LEFT_MOTORS) / sizeof(LEFT_MOTORS[0]);
static const size_t N_RIGHT = sizeof(RIGHT_MOTORS) / sizeof(RIGHT_MOTORS[0]);
static const uint8_t PWM_BITS = 8;

// arduino-esp32 changed its PWM API between 2.x and 3.x.
#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
static void pwmAttach(uint8_t pin, uint8_t) { ledcAttach(pin, PWM_HZ, PWM_BITS); }
static void pwmWrite(uint8_t pin, uint8_t, uint32_t duty) { ledcWrite(pin, duty); }
#else
static void pwmAttach(uint8_t pin, uint8_t channel) {
    ledcSetup(channel, PWM_HZ, PWM_BITS);
    ledcAttachPin(pin, channel);
}
static void pwmWrite(uint8_t, uint8_t channel, uint32_t duty) { ledcWrite(channel, duty); }
#endif

static void setChannel(const MotorChannel &m, uint8_t pwmChannel, int speed) {
    if (m.invert) speed = -speed;

    if (speed == 0) {
        digitalWrite(m.in1, HIGH);
        digitalWrite(m.in2, HIGH);      // short brake
        pwmWrite(m.pwm, pwmChannel, 0);
        return;
    }
    digitalWrite(m.in1, speed > 0 ? HIGH : LOW);
    digitalWrite(m.in2, speed > 0 ? LOW : HIGH);
    pwmWrite(m.pwm, pwmChannel, abs(speed));
}

static void initChannel(const MotorChannel &m, uint8_t pwmChannel) {
    pinMode(m.in1, OUTPUT);
    pinMode(m.in2, OUTPUT);
    pwmAttach(m.pwm, pwmChannel);
    setChannel(m, pwmChannel, 0);
}

bool DriveTrain::begin() {
    for (size_t i = 0; i < sizeof(STBY_PINS); i++) {
        pinMode(STBY_PINS[i], OUTPUT);
        digitalWrite(STBY_PINS[i], HIGH);
    }
    for (size_t i = 0; i < N_LEFT; i++) initChannel(LEFT_MOTORS[i], i);
    for (size_t i = 0; i < N_RIGHT; i++) initChannel(RIGHT_MOTORS[i], N_LEFT + i);
    return true;
}

void DriveTrain::drive(int left, int right) {
    _left = clampSpeed(left);
    _right = clampSpeed(right);
    for (size_t i = 0; i < N_LEFT; i++) setChannel(LEFT_MOTORS[i], i, _left);
    for (size_t i = 0; i < N_RIGHT; i++) setChannel(RIGHT_MOTORS[i], N_LEFT + i, _right);
}

// ===========================================================================
// SparkFun Qwiic Motor Driver (SCMD), I2C
//
// Boards chain together through their expansion ports. setDrive() takes a
// motor number, a direction (0/1) and a level (0-255).
// ===========================================================================
#elif DRIVE_BACKEND == DRIVE_QWIIC_SCMD

#include <SCMD.h>
#include <SCMD_config.h>

static SCMD scmd;
static const size_t N_LEFT = sizeof(LEFT_MOTOR_IDS) / sizeof(LEFT_MOTOR_IDS[0]);
static const size_t N_RIGHT = sizeof(RIGHT_MOTOR_IDS) / sizeof(RIGHT_MOTOR_IDS[0]);

static void setMotor(uint8_t id, bool invert, int speed) {
    if (invert) speed = -speed;
    scmd.setDrive(id, speed >= 0 ? 0 : 1, (uint8_t) abs(speed));
}

bool DriveTrain::begin() {
    scmd.settings.commInterface = I2C_MODE;
    scmd.settings.I2CAddress = SCMD_I2C_ADDR;

    // begin() returns the chip ID (0xA9) when the board answers.
    uint32_t start = millis();
    while (scmd.begin() != 0xA9) {
        if (millis() - start > 2000) return false;
        delay(50);
    }
    // Wait for the chain to finish enumerating its slave boards.
    while (!scmd.ready()) {
        if (millis() - start > 4000) return false;
        delay(10);
    }
    while (scmd.busy()) {
        if (millis() - start > 6000) return false;
        delay(10);
    }
    scmd.enable();
    drive(0, 0);
    return true;
}

void DriveTrain::drive(int left, int right) {
    _left = clampSpeed(left);
    _right = clampSpeed(right);
    for (size_t i = 0; i < N_LEFT; i++) setMotor(LEFT_MOTOR_IDS[i], LEFT_INVERT, _left);
    for (size_t i = 0; i < N_RIGHT; i++) setMotor(RIGHT_MOTOR_IDS[i], RIGHT_INVERT, _right);
}

#else
#error "config.h: DRIVE_BACKEND must be DRIVE_ACEBOTT_SHIELD, DRIVE_TB6612FNG or DRIVE_QWIIC_SCMD"
#endif
