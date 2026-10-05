#include <Arduino.h>
#include <Wire.h>

#include "Attitude.h"
#include "DriveTrain.h"
#include "GyroCalibrator.h"
#include "Mpu6050.h"
#include "Strategy.h"
#include "config.h"

// ---------------------------------------------------------------------------
// Monarch of the Hill
//
// Reads the MPU-6050 (gyro + accelerometer), works out which way is uphill,
// and drives the tracks to climb, stop mid-mesa and hold. The decision logic
// is in lib/Monarch (Attitude, Strategy) and is unit-tested on a PC with
// `pio test -e native`; this file only does the hardware plumbing.
//
// FIRST TIME ON THE REAL ROBOT - use the bench environment (esp32dev_bench):
// it never runs the strategy, it just prints sensor values and lets you drive
// from the serial monitor. Wire it per wire.md first, then work down this list:
//
//   1. Robot level on a table. pitch and roll should read near 0, tilt < 3.
//   2. Lift the NOSE. pitch must go positive.  If negative, flip the sign of
//      BODY_FORWARD in config.h (or fix the axis).
//   3. Lift the LEFT side (as seen from behind the robot). roll must go
//      positive.  If negative, flip the sign of BODY_LEFT.
//   4. Turn the robot counter-clockwise by hand (viewed from above). yawRate
//      must go positive.  If not, BODY_UP is pointing the wrong way.
//   5. Send 'l' then 'r' (one side at a time, tracks off the table). Each side
//      must drive FORWARD. Flip that side's invert flag in config.h if not.
//   6. (Only if you turn NUDGE_TO_CENTER on.) Send 'c': it drives straight at
//      CENTER_SPEED for exactly 2 seconds. Measure how far it went, in
//      inches, on flat ground. Put inches / 2.0 into MEASURED_SPEED_IN_PER_S
//      in config.h.
//   7. Flash the normal environment (esp32dev). HAND TEST before the ramp, with
//      the tracks off the table and the serial monitor open (it prints
//      phase, tilt, uphill bearing, yaw, left and right command):
//        - level: phase "waiting", tracks still.
//        - hold the nose up about 30 degrees: "slope", both tracks forward.
//        - hold the nose down about 30 degrees: "slope(rev)", both backward.
//        - hold it tilted with the left side up: the tracks turn opposite
//          ways, turning the nose toward the high side.
//        - tilt, then level it: the tracks stop and the phase becomes "holding"
//          (with NUDGE_TO_CENTER on it shows "centering" for a moment first).
//   8. Then the ramp. To aim for the middle of the mesa instead of stopping at
//      the first level reading, set NUDGE_TO_CENTER = true and tune
//      CENTER_OFFSET_IN (raise it if it stops short, lower it if past).
//
// Bench keys: w forward, s back, a spin left, d spin right, x stop,
//             l left side only, r right side only, c speed-measurement run.
// ---------------------------------------------------------------------------

// ---- tuning that depends on the real robot (see config.h) -----------------

static StrategyConfig makeStrategyConfig() {
    StrategyConfig c;
    c.driveSpeed = DRIVE_SPEED;
    c.crestSpeed = CREST_SPEED;
    c.centerSpeed = CENTER_SPEED;
    c.slowBelowTiltDeg = SLOW_BELOW_TILT_DEG;
    c.holdDeadbandDeg = HOLD_REAIM_DEG;
    c.centerNudge = NUDGE_TO_CENTER;
    c.measuredSpeedInPerS = MEASURED_SPEED_IN_PER_S;
    c.centerOffsetIn = CENTER_OFFSET_IN;
    return c;
}

// ---- globals ----------------------------------------------------------------

static Mpu6050 imu;
static DriveTrain drive;
static Attitude attitude;
static Strategy strategy(makeStrategyConfig());

static Vec3 gyroBias{0, 0, 0};
static float gyroSign = 1.0f;          // -1 if the axis map is a mirror image
static bool imuOk = false;

static const uint32_t IMU_STALE_MS = 250;   // no good reading for this long -> stop the motors

// ---- axis mapping -------------------------------------------------------------

static Vec3 toBodyAxes(const Vec3 &s) {
    return Vec3{BODY_FORWARD.sign * s.at(BODY_FORWARD.axis),
                BODY_LEFT.sign * s.at(BODY_LEFT.axis),
                BODY_UP.sign * s.at(BODY_UP.axis)};
}

// Accelerometer readings are polar vectors and gyro readings are axial ones:
// they only transform the same way under a proper rotation. A mapping that
// is a mirror image (determinant -1) needs the gyro negated to stay correct.
static bool checkAxisMap(float &gyroSignOut) {
    const AxisMap *m[3] = {&BODY_FORWARD, &BODY_LEFT, &BODY_UP};

    if (m[0]->axis == m[1]->axis || m[0]->axis == m[2]->axis || m[1]->axis == m[2]->axis) {
        return false;                               // an axis was used twice
    }
    float r[3][3] = {{0}};
    for (int i = 0; i < 3; i++) r[i][m[i]->axis] = (float) m[i]->sign;

    float det = r[0][0] * (r[1][1] * r[2][2] - r[1][2] * r[2][1])
                - r[0][1] * (r[1][0] * r[2][2] - r[1][2] * r[2][0])
                + r[0][2] * (r[1][0] * r[2][1] - r[1][1] * r[2][0]);
    gyroSignOut = det < 0 ? -1.0f : 1.0f;
    return true;
}

// ---- helpers ----------------------------------------------------------------

static void scanI2C() {
    int found = 0;
    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  device at 0x%02X\n", addr);
            found++;
        }
    }
    if (found == 0) Serial.println("  nothing on the bus - check wiring and pull-ups");
}

static void haltForever(const char *why) {
    drive.stop();
    for (;;) {
        Serial.println(why);
        delay(1000);
    }
}

// Averages the gyro while the robot sits still, and seeds the attitude
// estimate from the average accelerometer reading. Takes about half a second.
static void calibrateImu() {
    const int MIN_SAMPLES = 100;
    const float MAX_SPREAD_DPS = 1.0f;
    const uint32_t TIMEOUT_MS = 2000;

    GyroCalibrator cal;
    Vec3 accelSum{0, 0, 0};
    int accelN = 0;
    uint32_t start = millis();

    while (millis() - start < TIMEOUT_MS) {
        ImuSample s;
        if (imu.read(s)) {
            cal.add(toBodyAxes(s.gyroDps) * gyroSign);
            accelSum = accelSum + toBodyAxes(s.accelG);
            accelN++;
            if (cal.still(MIN_SAMPLES, MAX_SPREAD_DPS)) break;
        }
        delay(5);
    }

    if (accelN == 0) haltForever("IMU gave no readings during calibration");

    gyroBias = cal.bias();
    attitude.reset(accelSum * (1.0f / accelN));

    if (!cal.still(MIN_SAMPLES, MAX_SPREAD_DPS)) {
        Serial.printf("WARNING: robot was moving during gyro calibration (spread %.2f dps) - heading may drift\n",
                      cal.spreadDps());
    }
    Serial.printf("gyro bias: %.2f %.2f %.2f dps (%d samples)\n", gyroBias.x, gyroBias.y, gyroBias.z, cal.count());
}

// Reads the IMU, updates the attitude estimate. Returns true on a good reading.
static bool pollImu() {
    static uint32_t lastUs = 0;

    ImuSample s;
    if (!imu.read(s)) return false;

    uint32_t nowUs = micros();
    float dt = (nowUs - lastUs) * 1e-6f;
    lastUs = nowUs;
    if (dt <= 0.0f || dt > 0.05f) dt = IMU_PERIOD_MS * 1e-3f;     // first call, or a long gap

    Vec3 gyro = toBodyAxes(s.gyroDps) * gyroSign - gyroBias;
    attitude.update(toBodyAxes(s.accelG), gyro, dt);
    return true;
}

// ---------------------------------------------------------------------------

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("booted");

    if (!checkAxisMap(gyroSign)) {
        Serial.println("config.h: BODY_FORWARD / BODY_LEFT / BODY_UP must use three different sensor axes");
        for (;;) delay(1000);
    }

    if (!drive.begin()) Serial.println("WARNING: motor driver did not respond - check wiring");
    drive.stop();

    imuOk = imu.begin(MPU_I2C_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ);
    if (!imuOk) {
        Serial.printf("MPU-6050 not responding at 0x%02X (WHO_AM_I read 0x%02X) - scanning bus...\n",
                      MPU_I2C_ADDR, imu.whoAmI());
        scanI2C();
        haltForever("no IMU, motors held off");
    }
    Serial.printf("MPU-6050 ok (WHO_AM_I 0x%02X)\n", imu.whoAmI());

    calibrateImu();

#ifdef BENCH_MODE
    Serial.println("BENCH MODE: strategy disabled. See the checklist at the top of main.cpp.");
    Serial.println("keys: w s a d x l r c");
#else
    strategy.begin(millis());
    Serial.printf("center drive: %lu ms%s\n", (unsigned long) strategy.centerDriveMs(),
                  MEASURED_SPEED_IN_PER_S > 0.0f ? "" : "  (FALLBACK - MEASURED_SPEED_IN_PER_S not set)");
    Serial.println("phase\ttilt\tbearing\tyaw\tL\tR");
#endif
}

#ifdef BENCH_MODE

static const int BENCH_SPEED = 150;
static uint32_t benchRunUntilMs = 0;      // for the timed 'c' run

static void benchKeys() {
    const StrategyConfig c = makeStrategyConfig();
    while (Serial.available()) {
        switch (tolower(Serial.read())) {
            case 'w': drive.drive(BENCH_SPEED, BENCH_SPEED); break;
            case 's': drive.drive(-BENCH_SPEED, -BENCH_SPEED); break;
            case 'a': drive.drive(-BENCH_SPEED, BENCH_SPEED); break;
            case 'd': drive.drive(BENCH_SPEED, -BENCH_SPEED); break;
            case 'l': drive.drive(BENCH_SPEED, 0); break;
            case 'r': drive.drive(0, BENCH_SPEED); break;
            case 'x': drive.stop(); benchRunUntilMs = 0; break;
            case 'c':
                Serial.printf("driving at %d for 2.0 s - measure the distance\n", c.centerSpeed);
                drive.drive(c.centerSpeed, c.centerSpeed);
                benchRunUntilMs = millis() + 2000;
                break;
            default: break;
        }
    }
    if (benchRunUntilMs != 0 && (int32_t) (millis() - benchRunUntilMs) >= 0) {
        drive.stop();
        benchRunUntilMs = 0;
        Serial.println("done. MEASURED_SPEED_IN_PER_S = inches traveled / 2.0");
    }
}

void loop() {
    static uint32_t lastImu = 0, lastPrint = 0;
    uint32_t now = millis();

    if (now - lastImu >= IMU_PERIOD_MS) {
        lastImu = now;
        pollImu();
    }
    benchKeys();

    if (now - lastPrint >= 200) {
        lastPrint = now;
        Serial.printf("pitch %6.1f  roll %6.1f  tilt %5.1f  bearing %6.1f  yaw %7.1f  yawRate %6.1f  pitchRate %6.1f\n",
                      attitude.pitchDeg(), attitude.rollDeg(), attitude.tiltDeg(),
                      attitude.uphillBearingDeg(), attitude.yawDeg(),
                      attitude.yawRateDps(), attitude.pitchRateDps());
    }
}

#else   // normal run

void loop() {
    static uint32_t lastImu = 0, lastDrive = 0, lastPrint = 0, lastGoodImu = 0, lastRecover = 0;
    uint32_t now = millis();

    if (now - lastImu >= IMU_PERIOD_MS) {
        lastImu = now;
        if (pollImu()) lastGoodImu = now;
    }

    if (now - lastDrive >= DRIVE_PERIOD_MS) {
        lastDrive = now;

        if (now - lastGoodImu > IMU_STALE_MS) {
            // Blind: don't keep driving on a stale attitude. Try to bring the
            // chip back once a second (a brown-out resets it to sleep mode).
            drive.stop();
            if (now - lastRecover > 1000) {
                lastRecover = now;
                imu.begin(MPU_I2C_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_CLOCK_HZ);
            }
        } else {
            StrategyInputs in;
            in.nowMs = now;
            in.tiltDeg = attitude.tiltDeg();
            in.bearingDeg = attitude.uphillBearingDeg();
            in.yawDeg = attitude.yawDeg();
            in.yawRateDps = attitude.yawRateDps();
            DriveCommand cmd = strategy.update(in);
            drive.drive(cmd.left, cmd.right);
        }
    }

    if (now - lastPrint >= 250) {
        lastPrint = now;
        Serial.printf("%s%s\t%.1f\t%.0f\t%.0f\t%d\t%d\n",
                      strategy.phaseName(), strategy.reversing() ? "(rev)" : "",
                      attitude.tiltDeg(), attitude.uphillBearingDeg(), attitude.yawDeg(),
                      drive.lastLeft(), drive.lastRight());
    }
}

#endif
