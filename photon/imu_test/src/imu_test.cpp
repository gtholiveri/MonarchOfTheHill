#include "Particle.h"

// ---------------------------------------------------------------------------
// Photon 2 IMU bench test - temporary, not part of the robot.
//
// Reads the ADXL345 and the MPU6050 off the same I2C bus (SDA = D0, SCL = D1)
// and redraws one table in place, so the two can be compared at a glance.
// Sitting still, both should report |a| ~= 1.00 g, and the same tilt if they
// are mounted the same way up.
//
//   particle flash --local              (from this folder, Photon on USB)
//   particle serial monitor --follow
//
// Until both sensors answer it restarts the bus and retries every two
// seconds, printing which chip responded, so wiring can be fixed live.
//
// The table is redrawn with ANSI cursor codes. If the monitor shows them as
// garbage instead, set REDRAW_IN_PLACE to false and it scrolls.
//
// Both chips are read with plain Wire rather than libraries - waking each one
// and pulling a handful of bytes is all this needs. The motor driver is held
// asleep so nothing moves or buzzes while this runs.
// ---------------------------------------------------------------------------

// Connects to the cloud in the background so new code can be flashed over
// Wi-Fi. setup() and loop() start straight away and don't wait for it.
SYSTEM_MODE(AUTOMATIC);

static const unsigned long REFRESH_MS = 200;
static const unsigned long RETRY_MS = 2000;
static const bool REDRAW_IN_PLACE = true;

// Well under the usual 100 kHz. The ADXL345 breakout has no pull-ups of its
// own, and a bus with weak pull-ups rises slowly, which breaks register reads
// long before it breaks the address handshake. At 100 kHz the ADXL345 ACKed
// its address here but every read failed; at 20 kHz it reads cleanly.
static const uint32_t I2C_CLOCK_HZ = 20000;

static const pin_t PIN_MOTOR_STBY = D10;

static const uint8_t ADXL_ADDR = 0x53;              // SDO low
static const uint8_t ADXL_REG_DEVID = 0x00;
static const uint8_t ADXL_REG_POWER_CTL = 0x2D;
static const uint8_t ADXL_REG_DATA_FORMAT = 0x31;
static const uint8_t ADXL_REG_DATAX0 = 0x32;
static const uint8_t ADXL_DEVID_EXPECTED = 0xE5;
static const float ADXL_G_PER_COUNT = 0.0039f;      // full-resolution mode

static const uint8_t MPU_ADDR = 0x68;               // AD0 low
static const uint8_t MPU_ADDR_ALT = 0x69;           // AD0 high
static const uint8_t MPU_REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU_REG_WHO_AM_I = 0x75;

// Power-on full-scale ranges: +/-2 g and +/-250 deg/s.
static const float MPU_COUNTS_PER_G = 16384.0f;
static const float MPU_COUNTS_PER_DPS = 131.0f;

struct Vec3 {
    float x;
    float y;
    float z;

    float magnitude() const { return sqrtf(x * x + y * y + z * z); }
};

struct MpuSample {
    Vec3 accel;     // g
    Vec3 gyro;      // deg/s
    float tempC;
};

static bool adxlOk = false;
static bool mpuOk = false;
static uint8_t adxlId = 0;
static uint8_t mpuId = 0;

// --- I2C --------------------------------------------------------------------

static bool writeReg(uint8_t addr, uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

// endTransmission(false) is a repeated start: it keeps hold of the bus between
// setting the register pointer and reading back from it.
static bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;

    if (Wire.requestFrom(addr, len) != len) return false;
    for (size_t i = 0; i < len; i++) buf[i] = Wire.read();
    return true;
}

// Does anything ACK this address at all?
static bool answers(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

// --- ADXL345 ----------------------------------------------------------------

static bool adxlBegin() {
    adxlId = 0;
    if (!readRegs(ADXL_ADDR, ADXL_REG_DEVID, &adxlId, 1)) return false;
    if (adxlId != ADXL_DEVID_EXPECTED) return false;

    // FULL_RES keeps 3.9 mg per count; then leave standby and start measuring.
    return writeReg(ADXL_ADDR, ADXL_REG_DATA_FORMAT, 0x08) &&
           writeReg(ADXL_ADDR, ADXL_REG_POWER_CTL, 0x08);
}

static bool adxlSample(Vec3 &out) {
    uint8_t raw[6];     // x, y, z - a little-endian int16 each
    if (!readRegs(ADXL_ADDR, ADXL_REG_DATAX0, raw, sizeof(raw))) return false;

    out = Vec3{(int16_t) (raw[0] | (raw[1] << 8)) * ADXL_G_PER_COUNT,
               (int16_t) (raw[2] | (raw[3] << 8)) * ADXL_G_PER_COUNT,
               (int16_t) (raw[4] | (raw[5] << 8)) * ADXL_G_PER_COUNT};
    return true;
}

// --- MPU6050 ----------------------------------------------------------------

static int16_t bigEndian16(const uint8_t *p) {
    return (int16_t) ((p[0] << 8) | p[1]);
}

static bool mpuBegin() {
    mpuId = 0;
    if (!readRegs(MPU_ADDR, MPU_REG_WHO_AM_I, &mpuId, 1)) return false;

    // The chip powers up asleep. Writing 0x01 clears SLEEP and clocks it off
    // the X gyro, which the datasheet recommends over the internal oscillator.
    return writeReg(MPU_ADDR, MPU_REG_PWR_MGMT_1, 0x01);
}

static bool mpuSample(MpuSample &out) {
    uint8_t raw[14];    // accel xyz, temp, gyro xyz - a big-endian int16 each
    if (!readRegs(MPU_ADDR, MPU_REG_ACCEL_XOUT_H, raw, sizeof(raw))) return false;

    out.accel = Vec3{bigEndian16(raw + 0) / MPU_COUNTS_PER_G,
                     bigEndian16(raw + 2) / MPU_COUNTS_PER_G,
                     bigEndian16(raw + 4) / MPU_COUNTS_PER_G};
    out.tempC = bigEndian16(raw + 6) / 340.0f + 36.53f;
    out.gyro = Vec3{bigEndian16(raw + 8) / MPU_COUNTS_PER_DPS,
                    bigEndian16(raw + 10) / MPU_COUNTS_PER_DPS,
                    bigEndian16(raw + 12) / MPU_COUNTS_PER_DPS};
    return true;
}

// --- helpers ----------------------------------------------------------------

// Angle off the sensor's own +Z: 0 lying flat face-up, 90 on edge.
static float tiltDeg(const Vec3 &a) {
    float m = a.magnitude();
    if (m < 0.01f) return 0.0f;

    float cosine = constrain(a.z / m, -1.0f, 1.0f);
    return acosf(cosine) * 180.0f / (float) M_PI;
}

// --- table ------------------------------------------------------------------

static int rowsDrawn = 0;
static bool drawInPlace = false;

// One table row. ESC[K wipes whatever the previous frame left on the line, in
// case this row comes out shorter than the one it overwrites.
static void row(const char *fmt, ...) {
    char buf[96];

    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    Serial.print(buf);
    Serial.print(drawInPlace ? "\033[K\r\n" : "\r\n");
    rowsDrawn++;
}

static void accelRow(const char *name, bool ok, const Vec3 &g) {
    if (!ok) {
        row(" %-9s %9s", name, "no data");
        return;
    }
    row(" %-9s %+9.3f %+9.3f %+9.3f %9.3f %11.1f",
        name, g.x, g.y, g.z, g.magnitude(), tiltDeg(g));
}

static void drawTable(unsigned long now, bool inPlace) {
    Vec3 adxlG{0, 0, 0};
    bool adxlFresh = adxlOk && adxlSample(adxlG);

    MpuSample mpu{};
    bool mpuFresh = mpuOk && mpuSample(mpu);

    rowsDrawn = 0;
    drawInPlace = inPlace;

    row(" IMU bench test %38.1f s", now / 1000.0f);
    row("");
    row(" %-9s %9s %9s %9s %9s %11s", "", "x (g)", "y (g)", "z (g)", "|a| (g)", "tilt (deg)");
    accelRow("ADXL345", adxlFresh, adxlG);
    accelRow("MPU6050", mpuFresh, mpu.accel);
    row("");
    row(" %-9s %9s %9s %9s %9s", "", "x (deg/s)", "y (deg/s)", "z (deg/s)", "temp (C)");
    if (mpuFresh) {
        row(" %-9s %+9.2f %+9.2f %+9.2f %9.1f",
            "MPU gyro", mpu.gyro.x, mpu.gyro.y, mpu.gyro.z, mpu.tempC);
    } else {
        row(" %-9s %9s", "MPU gyro", "no data");
    }

    if (inPlace) {
        Serial.printf("\033[%dA", rowsDrawn);   // back to the top row for the next frame
    } else {
        Serial.println();
    }
}

// --- bus bring-up -----------------------------------------------------------

// Restarts the bus from scratch and tries both chips, reporting what each one
// said. Only the three addresses that matter are probed: a full scan stalls
// for seconds when a line is open.
static void bringUp(unsigned long now) {
    Wire.end();
    Wire.setSpeed(I2C_CLOCK_HZ);
    Wire.begin();

    adxlOk = adxlBegin();
    mpuOk = mpuBegin();

    Serial.printlnf("%.1f s   ADXL345 (0x53): %s   MPU6050 (0x68): %s%s", now / 1000.0f,
                    adxlOk ? "ok" : answers(ADXL_ADDR) ? "answers, read failed" : "no answer",
                    mpuOk ? "ok" : answers(MPU_ADDR) ? "answers, read failed" : "no answer",
                    !mpuOk && answers(MPU_ADDR_ALT) ? "   (something answers at 0x69)" : "");
}

void setup() {
    pinMode(PIN_MOTOR_STBY, OUTPUT);
    digitalWrite(PIN_MOTOR_STBY, LOW);

    Serial.begin(115200);

    delay(1000);
    Serial.println();
    Serial.println("IMU bench test");
}

void loop() {
    static unsigned long lastDraw = 0;
    static unsigned long lastRetry = 0;
    static bool everTried = false;

    unsigned long now = millis();
    bool allOk = adxlOk && mpuOk;

    if (!allOk && (!everTried || now - lastRetry >= RETRY_MS)) {
        everTried = true;
        lastRetry = now;

        bringUp(now);
        if (adxlOk || mpuOk) drawTable(now, false);
        return;
    }

    if (allOk && now - lastDraw >= REFRESH_MS) {
        lastDraw = now;
        drawTable(now, REDRAW_IN_PLACE);
    }
}
