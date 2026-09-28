#include <Arduino.h>
#include <Wire.h>

#include "Accelerometer.h"

// ---------------------------------------------------------------------------
// ACEBOTT ESP32 Car Shield V2.0 - motor interface
//
// The V2.0 shield does NOT expose the H-bridge to the ESP32. A motor
// co-processor on the shield does the switching, and the ESP32 drives it over
// a 9600-baud serial link. One command is a 5-byte packet:
//
//     byte 0   motor index   1-4
//     byte 1   direction     1 = forward, 2 = reverse
//     byte 2   speed         0-100
//     byte 3   '\r'
//     byte 4   '\n'
//
// GPIO16/17 are RX/TX - the ESP32's default Serial2 pins, which is why ACEBOTT
// chose them. Their own library bit-bangs this with SoftwareSerial; hardware
// UART2 is used here instead.
//
// Quirks inherited from ACEBOTT's library, kept deliberately:
//   - Motors 1/2 and 3/4 have MIRRORED direction conventions, because they are
//     mounted facing opposite ways on the two sides of the chassis. Positive
//     speed to M1 and M2 spins them the same rotational direction.
//   - map() lands on 40 at an input of ~102, which the protocol treats as stop.
//     A speed ramp passing through 102 will briefly cut the motor.
// ---------------------------------------------------------------------------

static const int MOTOR_RX_PIN = 16;
static const int MOTOR_TX_PIN = 17;
static const long MOTOR_BAUD = 9600;

static Accelerometer accel;
static bool accelOk = false;

// --- tilt-driving behaviour -------------------------------------------------
//
// Drive forward while climbing, stop on the flat mesa, and reverse back onto
// it if the vehicle carries past the far edge.
//
// A magnitude-only tilt reading (angle off vertical, no sign) cannot tell
// this apart from "climbing the near ramp": both read the same angle. So
// pitch here is SIGNED - positive nose-up (climbing), negative nose-down
// (descending the far ramp after overshooting) - computed from two axes
// instead of the angle-to-vertical trick used previously.
//
// signedPitchDeg() below assumes the sensor's local X axis points toward the
// front of the vehicle and Z points up when the chassis is level. If the
// sign comes out backwards on the bench (drives fine uphill but reverses the
// wrong way after an overshoot), swap which axis feeds atan2f() in there.
//
// The tilt sensor also cannot tell WHERE on the 1 ft mesa the vehicle is -
// it reads "flat" the whole way across. So once flat is detected, we keep
// driving forward for a calculated time instead of stopping immediately:
// dead reckoning, distance = speed x time, so time = distance / speed. The
// distance is half the mesa (known, from the contest rules). The speed has
// to come from you timing the real vehicle - see centerDriveMs() below for
// exactly how.

static const unsigned long SAMPLE_MS = 20;    // 50 Hz
static const unsigned long PRINT_MS = 500;

static const float EMA_ALPHA = 0.2f;          // ~0.1 s to settle at 50 Hz

static const float PITCH_THRESHOLD_DEG = 15.0f;  // magnitude; climb above +this, overshoot below -this

static const int DRIVE_SPEED = 200;

// --- dead reckoning for the centering nudge ---------------------------------
//
// How to measure MEASURED_SPEED_IN_PER_S:
//   1. On a flat floor, mark a straight 24 inch line.
//   2. Power up so it drives at DRIVE_SPEED (same speed used for the climb),
//      starting at the mark, and time it with a stopwatch until the front of
//      the vehicle crosses the 24 inch mark.
//   3. Set MEASURED_SPEED_IN_PER_S = 24.0 / (seconds you measured).
//      A longer run (24 in rather than 6 in) makes stopwatch reaction time a
//      smaller fraction of the error.
//
// Left at 0, centerDriveMs() falls back to FALLBACK_CENTER_DRIVE_MS instead
// of dividing by zero, so the robot still does *something* reasonable before
// you've measured a real speed.
//
// This assumes flat is detected right as the vehicle reaches the mesa. In
// practice the chassis may already be partway onto the mesa by the time it
// reads level, since the sensor isn't at the very front. If testing shows it
// consistently lands short of or past center by roughly the same amount
// every time, adjust CENTER_OFFSET_IN to correct for it - positive drives a
// bit further, negative a bit less.

static const float MESA_LENGTH_IN = 12.0f;             // full flat top, from the contest rules
static const float MEASURED_SPEED_IN_PER_S = 0.0f;     // TODO: measure, see above
static const float CENTER_OFFSET_IN = 0.0f;             // TODO: adjust after real-ramp testing
static const unsigned long FALLBACK_CENTER_DRIVE_MS = 300;  // used only until speed is measured

static unsigned long centerDriveMs() {
    if (MEASURED_SPEED_IN_PER_S <= 0.0f) return FALLBACK_CENTER_DRIVE_MS;

    float distanceIn = (MESA_LENGTH_IN / 2.0f) + CENTER_OFFSET_IN;
    float seconds = distanceIn / MEASURED_SPEED_IN_PER_S;
    if (seconds < 0.0f) seconds = 0.0f;
    return (unsigned long) (seconds * 1000.0f);
}

// Safety cap so a reverse-out-of-overshoot can't run forever if the pitch
// reading never settles back to flat (e.g. it drove off the ramp entirely).
static const unsigned long MAX_REVERSE_MS = 1500;

// How long pitch must stay flat before we trust "we've stopped tilting"
// (mid-climb bumps and sensor noise near the threshold otherwise flicker
// this back and forth).
static const unsigned long FLAT_CONFIRM_MS = 150;

// Same idea for the start of the climb, so a bump at power-on isn't
// mistaken for actually being on the ramp.
static const unsigned long CLIMB_CONFIRM_MS = 100;

static Vec3 smoothed{0, 0, 0};

enum ClimbState { WAIT_CLIMB, CLIMBING, CENTERING, STOPPED, REVERSING };
static ClimbState state = WAIT_CLIMB;
static unsigned long stateEnteredAt = 0;
static unsigned long flatSince = 0;
static unsigned long climbSince = 0;

// Reports every address that ACKs, so a miswired or differently-addressed
// sensor shows up instead of just going quiet.
void scanI2C() {
    int found = 0;

    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.printf("  device at 0x%02X\n", addr);
            found++;
        }
    }

    if (found == 0) Serial.println("  nothing on the bus - check wiring/pull-ups");
}

// speed: -255..255. Sign sets direction, magnitude sets speed. 0 stops.
void motorControl(uint8_t motor, int speed) {
    if (motor < 1 || motor > 4) return;

    speed = constrain(speed, -255, 255);

    uint8_t dir = (motor <= 2)
                      ? (speed > 0 ? 1 : 2)
                      : (speed > 0 ? 2 : 1);

    uint8_t speed_value = map(abs(speed), 0, 255, 0, 100);
    if (speed_value == 40) speed_value = 0; // protocol quirk, see above

    Serial2.write(motor);
    Serial2.write(dir);
    Serial2.write(speed_value);
    Serial2.write('\r');
    Serial2.write('\n');
}

void stopAll() {
    for (uint8_t m = 1; m <= 4; m++) motorControl(m, 0);
}

void driveForward(int speed) {
    for (uint8_t m = 1; m <= 4; m++) motorControl(m, speed);
}

void driveReverse(int speed) {
    driveForward(-abs(speed));
}

// --- vector helpers ---------------------------------------------------------

static Vec3 ema(const Vec3 &prev, const Vec3 &sample, float alpha) {
    return Vec3{prev.x + alpha * (sample.x - prev.x),
                prev.y + alpha * (sample.y - prev.y),
                prev.z + alpha * (sample.z - prev.z)};
}

// Signed pitch, in degrees. Positive = nose up (climbing the near ramp).
// Negative = nose down (descending the far ramp after an overshoot).
// Assumes the sensor's local X axis points to the front of the vehicle and Z
// points up when the chassis is level - confirm this on the bench (see the
// note above the state machine) before trusting the sign.
static float signedPitchDeg(const Vec3 &g) {
    return degrees(atan2f(g.x, g.z));
}

void setup() {
    Serial.begin(115200); // main serial monitor
    Serial2.begin(MOTOR_BAUD, SERIAL_8N1, MOTOR_RX_PIN, MOTOR_TX_PIN); // send specific serial bytes to relevant pins on shield
    delay(200);

    stopAll();

    delay(100);
    // motorControl(1, 255);
    // motorControl(4, 255);

    // Banner first: anything that touches the I2C bus can stall, and a silent
    // boot is impossible to tell apart from a dead upload.
    Serial.println();
    Serial.println("booted");
    if (MEASURED_SPEED_IN_PER_S <= 0.0f) {
        Serial.println("center drive: no speed measured yet, using fallback - see centerDriveMs()");
    }
    Serial.printf("center drive: %lu ms\n", centerDriveMs());

    accelOk = accel.begin();

    if (accelOk) {
        Serial.println("ADXL345 ok");

        // Seed the filter with a real reading, otherwise it ramps up from
        // zero and reports a bogus 0 deg for the first fraction of a second.
        smoothed = accel.read();
        Serial.printf("start: %.2f %.2f %.2f  pitch=%.1f deg\n",
                      smoothed.x, smoothed.y, smoothed.z,
                      signedPitchDeg(smoothed));

        Serial.println("pitch(deg)\tstate");
    } else {
        Serial.println("ADXL345 not responding at 0x53 - scanning bus...");
        scanI2C();
    }
}

static void enterState(ClimbState next, unsigned long now) {
    state = next;
    stateEnteredAt = now;
}

static const char *stateName(ClimbState s) {
    switch (s) {
        case WAIT_CLIMB: return "wait_climb";
        case CLIMBING:   return "climbing";
        case CENTERING:  return "centering";
        case STOPPED:    return "stopped";
        case REVERSING:  return "reversing";
    }
    return "?";
}

void loop() {
    static unsigned long lastSample = 0;
    static unsigned long lastPrint = 0;

    if (!accelOk) return;

    unsigned long now = millis();
    if (now - lastSample < SAMPLE_MS) return;
    lastSample = now;

    smoothed = ema(smoothed, accel.read(), EMA_ALPHA);
    float pitch = signedPitchDeg(smoothed);

    bool climbingTilt = pitch >= PITCH_THRESHOLD_DEG;
    bool overshootTilt = pitch <= -PITCH_THRESHOLD_DEG;
    bool flatTilt = !climbingTilt && !overshootTilt;

    if (flatTilt) {
        if (flatSince == 0) flatSince = now;
    } else {
        flatSince = 0;
    }
    bool confirmedFlat = flatTilt && flatSince != 0 && (now - flatSince >= FLAT_CONFIRM_MS);

    if (climbingTilt) {
        if (climbSince == 0) climbSince = now;
    } else {
        climbSince = 0;
    }
    bool confirmedClimb = climbingTilt && climbSince != 0 && (now - climbSince >= CLIMB_CONFIRM_MS);

    // Overshoot can be reached from any driving state - check it first,
    // regardless of what we were doing when it happened.
    if (overshootTilt && state != REVERSING && state != WAIT_CLIMB) {
        driveReverse(DRIVE_SPEED);
        enterState(REVERSING, now);
    }

    switch (state) {

        case WAIT_CLIMB:
            if (confirmedClimb) {
                driveForward(DRIVE_SPEED);
                enterState(CLIMBING, now);
            }
            break;

        case CLIMBING:
            if (confirmedFlat) {
                // Reached the mesa. Keep driving a bit longer to reach the
                // middle instead of stopping right at the near edge - see
                // the dead-reckoning note above the state machine.
                enterState(CENTERING, now);
            }
            break;

        case CENTERING:
            if (now - stateEnteredAt >= centerDriveMs()) {
                stopAll();
                enterState(STOPPED, now);
            }
            break;

        case STOPPED:
            // Sit tight. The overshoot check above will pull us out of this
            // if we get bumped past the far edge.
            break;

        case REVERSING:
            if (confirmedFlat || now - stateEnteredAt >= MAX_REVERSE_MS) {
                stopAll();
                enterState(STOPPED, now);
            }
            break;
    }

    if (now - lastPrint >= PRINT_MS) {
        lastPrint = now;
        Serial.printf("%.1f\t\t%s\n", pitch, stateName(state));
    }
}
