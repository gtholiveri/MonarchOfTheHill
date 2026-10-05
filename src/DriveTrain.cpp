#include "DriveTrain.h"

#include "config.h"

// SparkFun Motor Driver - Dual TB6612FNG.
//
// Per motor: the PWM pin sets speed, IN1/IN2 set direction.
//   IN1 H, IN2 L -> one way      IN1 L, IN2 H -> the other way
//   IN1 H, IN2 H -> short brake  (used for speed 0: resists being pushed)
// STBY must be HIGH or the chip ignores everything. The board pulls it low
// until we raise it, so the motors cannot move before this code runs.

static const uint8_t PWM_BITS = 8;
static const uint8_t LEFT_PWM_CHANNEL = 0;      // ESP32 LEDC channels (arduino-esp32 2.x)
static const uint8_t RIGHT_PWM_CHANNEL = 1;

static int clampSpeed(int v) { return v < -255 ? -255 : (v > 255 ? 255 : v); }

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

static void setMotor(const MotorChannel &m, uint8_t pwmChannel, int speed) {
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

static void initMotor(const MotorChannel &m, uint8_t pwmChannel) {
    pinMode(m.in1, OUTPUT);
    pinMode(m.in2, OUTPUT);
    pwmAttach(m.pwm, pwmChannel);
    setMotor(m, pwmChannel, 0);
}

bool DriveTrain::begin() {
    initMotor(LEFT_MOTOR, LEFT_PWM_CHANNEL);
    initMotor(RIGHT_MOTOR, RIGHT_PWM_CHANNEL);
    pinMode(STBY_PIN, OUTPUT);
    digitalWrite(STBY_PIN, HIGH);       // enable only after both motors are held in brake
    return true;
}

void DriveTrain::drive(int left, int right) {
    _left = clampSpeed(left);
    _right = clampSpeed(right);
    setMotor(LEFT_MOTOR, LEFT_PWM_CHANNEL, _left);
    setMotor(RIGHT_MOTOR, RIGHT_PWM_CHANNEL, _right);
}
