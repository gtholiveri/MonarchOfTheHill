#ifndef MONARCH_CONFIG_H
#define MONARCH_CONFIG_H

#include <Arduino.h>

// ===========================================================================
// Everything that depends on how this robot is built. Pins match wire.md.
// If something behaves wrong on the real robot, it is almost certainly one of
// the values below, not the logic in lib/Monarch.
// ===========================================================================

// ---------------------------------------------------------------------------
// 1. MPU-6050 (gyro + accelerometer) over I2C.
// ---------------------------------------------------------------------------
static const uint8_t MPU_I2C_ADDR = 0x68;      // 0x69 if the AD0 pin is pulled high
static const int I2C_SDA_PIN = 21;
static const int I2C_SCL_PIN = 22;
static const uint32_t I2C_CLOCK_HZ = 100000;   // drop to 50000 if reads time out (weak pull-ups)

// Which sensor axis points where on the ROBOT. Body frame is X = forward,
// Y = left, Z = up. With the chip flat and its printed X arrow toward the
// front, these defaults are right. If the bench checks in main.cpp show a
// sign backwards, flip that sign here.
//   axis: 0 = sensor X, 1 = sensor Y, 2 = sensor Z.   sign: +1 or -1.
struct AxisMap {
    uint8_t axis;
    int8_t sign;
};
static const AxisMap BODY_FORWARD = {0, +1};
static const AxisMap BODY_LEFT = {1, +1};
static const AxisMap BODY_UP = {2, +1};

// ---------------------------------------------------------------------------
// 2. Drivetrain: one SparkFun TB6612FNG board, channel A = left motor,
// channel B = right motor.
//
// invert = true flips a motor that spins backward for a forward command. The
// two motors face opposite ways on the chassis, so one of them usually needs
// it. The bench keys 'l' and 'r' (see main.cpp) show which.
// ---------------------------------------------------------------------------
struct MotorChannel {
    uint8_t pwm;
    uint8_t in1;
    uint8_t in2;
    bool invert;
};
static const MotorChannel LEFT_MOTOR = {25, 26, 27, false};    // PWMA, AIN1, AIN2
static const MotorChannel RIGHT_MOTOR = {32, 33, 13, true};    // PWMB, BIN1, BIN2
static const uint8_t STBY_PIN = 4;
static const uint32_t PWM_HZ = 20000;          // above hearing range; the chip allows up to 100 kHz

// ---------------------------------------------------------------------------
// 3. Calibration. These two numbers can only come from the real robot.
//
// The tilt sensors cannot say WHERE on the 1 ft mesa the robot is, so after it
// first reads level it drives for a calculated time:
//     time = (half the mesa + CENTER_OFFSET_IN) / MEASURED_SPEED_IN_PER_S
//
// MEASURED_SPEED_IN_PER_S: robot speed in inches per second at the nudge speed
//   (CENTER_SPEED above). Measure it with the bench 'c' key (main.cpp).
//   0 means "not measured yet"; the robot then uses a fixed 300 ms nudge and
//   says so on the serial monitor at boot.
// CENTER_OFFSET_IN: the chassis only reads level once it is already part-way
//   onto the mesa, so the nudge should be a little shorter than half the mesa.
//   -2 comes from simulating an 8 to 10 inch chassis. On the real ramp: if it
//   stops short of center, raise it; if past, lower it.
// ---------------------------------------------------------------------------
static const float MEASURED_SPEED_IN_PER_S = 0.0f;
static const float CENTER_OFFSET_IN = -2.0f;

// ---------------------------------------------------------------------------
// 4. Speeds, on a scale of 0 to 255 (255 = full battery voltage on the motors).
// These are the knobs to turn on the ramp.
//
// DRIVE_SPEED          climbing, while the chassis is steeply tilted
// CREST_SPEED          used once the tilt falls below SLOW_BELOW_TILT_DEG, to
//                      ease over the top instead of flying across the mesa.
//                      If the robot stalls near the crest, raise this.
// CENTER_SPEED         the nudge to mid-mesa. MEASURED_SPEED_IN_PER_S must be
//                      measured at THIS speed.
// HOLD_REAIM_DEG       when holding the mesa, turn back toward the original
//                      heading if pushed around by more than this many
//                      degrees. 0 turns re-aiming off.
// ---------------------------------------------------------------------------
static const int DRIVE_SPEED = 200;
static const int CREST_SPEED = 120;
static const int CENTER_SPEED = 120;
static const float SLOW_BELOW_TILT_DEG = 26.0f;
static const float HOLD_REAIM_DEG = 20.0f;

// ---------------------------------------------------------------------------
// 5. Control loop timing.
// ---------------------------------------------------------------------------
static const uint32_t IMU_PERIOD_MS = 10;       // 100 Hz sensor fusion
static const uint32_t DRIVE_PERIOD_MS = 40;     // 25 Hz strategy + motor updates

#endif // MONARCH_CONFIG_H
