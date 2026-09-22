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
// Drive forward whenever the chassis is tilted past a threshold off vertical,
// stop when it is back under it.
//
// Tilt is the angle between the smoothed gravity vector and +Z, which holds
// because the mounting fixes the sensor's Z axis pointing up.

static const unsigned long SAMPLE_MS = 20;    // 50 Hz
static const unsigned long PRINT_MS = 500;

static const float EMA_ALPHA = 0.2f;          // ~0.1 s to settle at 50 Hz

static const float TILT_THRESHOLD_DEG = 30.0f;   // at or past this -> drive

static const int DRIVE_SPEED = 200;

// Upright reference. Only the direction matters - angleBetweenDeg normalises.
// If the board reads ~180 deg sitting flat, the sensor's Z points down and
// this wants to be {0, 0, -1}.
static const Vec3 UP{0, 0, 1};

static Vec3 smoothed{0, 0, 0};
static bool driving = false;

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

// --- vector helpers ---------------------------------------------------------

static Vec3 ema(const Vec3 &prev, const Vec3 &sample, float alpha) {
    return Vec3{prev.x + alpha * (sample.x - prev.x),
                prev.y + alpha * (sample.y - prev.y),
                prev.z + alpha * (sample.z - prev.z)};
}

// Angle between two vectors, in degrees.
static float angleBetweenDeg(const Vec3 &a, const Vec3 &b) {
    float ma = a.magnitude();
    float mb = b.magnitude();
    if (ma < 0.01f || mb < 0.01f) return 0.0f;

    float cosine = (a.x * b.x + a.y * b.y + a.z * b.z) / (ma * mb);
    cosine = constrain(cosine, -1.0f, 1.0f);    // guard acos against rounding
    return degrees(acosf(cosine));
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

    accelOk = accel.begin();

    if (accelOk) {
        Serial.println("ADXL345 ok");

        // Seed the filter with a real reading, otherwise it ramps up from
        // zero and reports a bogus 0 deg for the first fraction of a second.
        smoothed = accel.read();
        Serial.printf("start: %.2f %.2f %.2f  tilt=%.1f deg\n",
                      smoothed.x, smoothed.y, smoothed.z,
                      angleBetweenDeg(smoothed, UP));

        Serial.println("tilt(deg)\tdriving");
    } else {
        Serial.println("ADXL345 not responding at 0x53 - scanning bus...");
        scanI2C();
    }
}

void loop() {
    static unsigned long lastSample = 0;
    static unsigned long lastPrint = 0;

    if (!accelOk) return;

    unsigned long now = millis();
    if (now - lastSample < SAMPLE_MS) return;
    lastSample = now;

    smoothed = ema(smoothed, accel.read(), EMA_ALPHA);
    float tilt = angleBetweenDeg(smoothed, UP);

    // Only send on a state change. The shield link is 9600 baud and each
    // update is 4 motors x 5 bytes = 20 bytes; at 50 Hz that is 1000 B/s
    // against a ~960 B/s budget, so resending every sample would back the
    // serial buffer up and the motors would lag behind the tilt.
    bool shouldDrive = tilt >= TILT_THRESHOLD_DEG;

    if (shouldDrive != driving) {
        driving = shouldDrive;
        if (driving) {
            driveForward(DRIVE_SPEED);
        } else {
            stopAll();
        }
    }

    if (now - lastPrint >= PRINT_MS) {
        lastPrint = now;
        Serial.printf("%.1f\t\t%s\n", tilt, driving ? "yes" : "no");
    }
}
