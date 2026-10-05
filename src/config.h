#ifndef MONARCH_CONFIG_H
#define MONARCH_CONFIG_H

#include <Arduino.h>

// ===========================================================================
// Everything that depends on how the robot is physically built lives here.
// If something behaves wrong on the real robot, it is almost certainly one of
// the values below, not the logic in lib/Monarch.
// ===========================================================================

// ---------------------------------------------------------------------------
// 1. Which motor driver board is attached. Pick exactly one.
//
//   DRIVE_ACEBOTT_SHIELD   the original ACEBOTT car shield (serial protocol)
//   DRIVE_TB6612FNG        SparkFun Motor Driver - Dual TB6612FNG
//                          (PWM + IN1 + IN2 pins per motor, one STBY pin)
//   DRIVE_QWIIC_SCMD       SparkFun Qwiic Motor Driver (I2C, daisy-chainable)
//
// Can also be set per environment with build_flags = -DDRIVE_BACKEND=...
// ---------------------------------------------------------------------------
#define DRIVE_ACEBOTT_SHIELD 1
#define DRIVE_TB6612FNG 2
#define DRIVE_QWIIC_SCMD 3

#ifndef DRIVE_BACKEND
#define DRIVE_BACKEND DRIVE_TB6612FNG
#endif

// ---------------------------------------------------------------------------
// 2. MPU-6050 (gyro + accelerometer) over I2C.
// ---------------------------------------------------------------------------
static const uint8_t MPU_I2C_ADDR = 0x68;      // 0x69 if the AD0 pin is pulled high
static const int I2C_SDA_PIN = 21;             // ESP32 default
static const int I2C_SCL_PIN = 22;             // ESP32 default
static const uint32_t I2C_CLOCK_HZ = 100000;   // drop to 50000 if reads time out (weak pull-ups)

// Which sensor axis points where on the ROBOT. Body frame is X = forward,
// Y = left, Z = up. If the chip is mounted flat with its printed X arrow
// toward the front, leave these alone. Otherwise change them - the bench
// checklist at the top of main.cpp tells you how to confirm.
//   axis: 0 = sensor X, 1 = sensor Y, 2 = sensor Z.   sign: +1 or -1.
struct AxisMap {
    uint8_t axis;
    int8_t sign;
};
static const AxisMap BODY_FORWARD = {0, +1};
static const AxisMap BODY_LEFT = {1, +1};
static const AxisMap BODY_UP = {2, +1};

// ---------------------------------------------------------------------------
// 3. Drivetrain wiring (which motors are on which side).
//
// Each side has one or two motors. "invert" flips a motor that spins the
// wrong way for a positive command (common when the motors are mounted
// mirrored on the two sides). The bench mode lets you test each side.
// ---------------------------------------------------------------------------

#if DRIVE_BACKEND == DRIVE_TB6612FNG
// TODO: replace every pin number with what is actually wired on your robot.
// (Avoid GPIO 0, 2, 12, 15 - the ESP32 reads them at boot - and 6-11, 21, 22,
// 34-39. 21/22 are the I2C pins used by the MPU-6050.)
// One TB6612FNG board drives two motors (channels A and B). Four motors need
// two boards: give the second board its own pins and STBY.
struct MotorChannel {
    uint8_t pwm;
    uint8_t in1;
    uint8_t in2;
    bool invert;
};
static const MotorChannel LEFT_MOTORS[] = {
    {25, 26, 27, false},       // TODO left motor 1: PWM, IN1, IN2
    {32, 33, 14, false},       // TODO left motor 2 (delete this line if one motor per side)
};
static const MotorChannel RIGHT_MOTORS[] = {
    {13, 18, 19, false},       // TODO right motor 1
    {4, 5, 23, false},         // TODO right motor 2 (delete if one motor per side)
};
static const uint8_t STBY_PINS[] = {16, 17};   // TODO one per TB6612FNG board
static const uint32_t PWM_HZ = 20000;          // above hearing range; TB6612FNG allows up to 100 kHz
#endif

#if DRIVE_BACKEND == DRIVE_QWIIC_SCMD
// The motor driver boards chain together; the first (master) answers at 0x5D.
// Motor numbers: master board = 0 and 1, the next board in the chain = 2 and 3.
static const uint8_t SCMD_I2C_ADDR = 0x5D;
static const uint8_t LEFT_MOTOR_IDS[] = {0, 2};      // TODO
static const uint8_t RIGHT_MOTOR_IDS[] = {1, 3};     // TODO
static const bool LEFT_INVERT = false;               // TODO flip if the left side runs backward
static const bool RIGHT_INVERT = true;               // TODO flip if the right side runs backward
#endif

// ---------------------------------------------------------------------------
// 4. Mesa dead reckoning. The tilt sensors cannot say WHERE on the 1 ft mesa
// the robot is, so after it first reads level it drives for a calculated time:
//     time = (half the mesa + CENTER_OFFSET_IN) / MEASURED_SPEED_IN_PER_S
//
// MEASURED_SPEED_IN_PER_S: leave at 0 until measured (a fixed fallback time is
//   used instead). Measure it with the bench 'c' key - see main.cpp.
// CENTER_OFFSET_IN: the chassis only reads level once it is already part-way
//   onto the mesa, so the nudge needs to be a little shorter than half the
//   mesa. -2 comes from simulating an 8-10 in chassis; confirm on the real
//   ramp. If it stops short of center consistently, raise it; if past, lower.
// ---------------------------------------------------------------------------
static const float MEASURED_SPEED_IN_PER_S = 0.0f;   // TODO measure
static const float CENTER_OFFSET_IN = -2.0f;

// ---------------------------------------------------------------------------
// 5. Control loop timing.
// ---------------------------------------------------------------------------
static const uint32_t IMU_PERIOD_MS = 10;       // 100 Hz sensor fusion
static const uint32_t DRIVE_PERIOD_MS = 40;     // 25 Hz strategy + motor updates
                                                // (the ACEBOTT serial link can't take faster)

#endif // MONARCH_CONFIG_H
