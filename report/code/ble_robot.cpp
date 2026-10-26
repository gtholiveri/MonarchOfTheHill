#include "Particle.h"

#include "Autopilot.h"
#include "Capture.h"
#include "Estimator.h"

// ---------------------------------------------------------------------------
// Photon 2 Bluetooth robot - manual driving, the autonomous run, telemetry.
//
// Pairs with dashboard/index.html, which talks to it from Chrome or Edge over
// Web Bluetooth. The link is the Nordic UART service, the de-facto standard
// for "serial over BLE": the browser writes to RX, the robot notifies on TX.
//
//   particle flash p2348                (from this folder, over Wi-Fi)
//
// Commands, as text lines written to RX:
//
//     D,<left>,<right>\n     drive; each -255..255, sign sets direction
//     S\n                    brake both motors, and end an autonomous run
//     A\n                    start the autonomous run (see Autopilot.h)
//     ARM\n                  wait for the start signal on the robot itself
//     P,<id>,<value>\n       set autopilot parameter <id> (ParamId order)
//     Z\n                    zero the gyro (robot must be sitting still)
//     L\n                    robot is level right now: this direction is up
//     U\n                    nose is tilted up right now: learn which way is forward
//     H,<0|1>\n              high-rate capture off or on (see Capture.h)
//     HS\n, HE\n             begin or end a capture by hand, for a recording
//                            that isn't an autonomous run
//     G,<first>,<count>,<ms>\n   send captured samples first .. first+count-1,
//                            one every <ms> milliseconds
//
// The dashboard resends D about ten times a second while a key is held. If
// COMMAND_TIMEOUT_MS passes without one - link dropped, laptop asleep, page
// closed - the motors brake by themselves. During an autonomous run a D with
// any drive in it takes over from the autopilot; an all-zero D is ignored, so
// the idle stream doesn't cancel the run.
//
// Packets on TX, all little-endian. Twenty bytes is what fits in one
// notification at the smallest BLE packet size, so none is ever split.
//
//   Estimate, 20 bytes, every TELEMETRY_MS:
//     0      'E'
//     1      flags: bit0 ADXL345 present, bit1 MPU6050 ok, bit2 motors driven,
//                   bit3 accelerometer trusted this tick, bit4 forward known,
//                   bit5 gyro zeroing under way
//     2-7    acceleration x, y, z       int16, thousandths of a g
//     8-13   rotation rate x, y, z      int16, tenths of a deg/s, offset removed
//     14-15  pitch, nose up positive    int16, hundredths of a degree
//     16-17  roll, left up positive     int16, hundredths of a degree
//     18-19  pitch from the accelerometer alone, same units, for comparison
//
//   Status, 10 bytes, every STATUS_MS:
//     0      'R'
//     1      System.resetReason() - why the Photon last restarted
//     2-5    System.resetReasonData()   uint32, the panic code after a crash
//     6-9    System.uptime()            uint32, seconds since that restart
//
//   Calibration, 13 bytes, every STATUS_MS:
//     0      'C'
//     1-6    the robot's up direction in sensor axes      int16 x 10000
//     7-12   the robot's forward direction in sensor axes int16 x 10000
//
//   Autopilot, 13 bytes, every TELEMETRY_MS:
//     0      'A'
//     1      AutoState: 0 idle, 1 climb, 2 brake, 3 hold, 4 armed
//     2      EndReason of the last run
//     3-4    time in this state         uint16, hundredths of a second
//     5-6    time in this run           uint16, hundredths of a second
//     7-8    heading change since start int16, tenths of a degree, left positive
//     9      autopilot's left output    int8, power / 2
//     10     autopilot's right output   int8, power / 2
//     11     parameters received since power-up (wraps), to confirm a push
//     12     bit0 start switch reads on, bit1 launch detector settled and ready
//
//   Motors and power, 13 bytes, every TELEMETRY_MS. Sent last, so it closes
//   the tick: the dashboard logs one row per M, using the E and A just before.
//     0      'M'
//     1-4    millis()                   uint32, the robot's clock
//     5-6    channel A duty applied     int16, -255..255, after trim, side
//                                       swap and easing; what the driver gets
//     7-8    channel B duty applied     int16
//     9-10   pack voltage               uint16, millivolts
//     11-12  pack sense pin, raw        uint16, ADC counts 0..4095
//
//   Capture status, 11 bytes, every STATUS_MS, as soon as a log is finished,
//   and when a G request has been answered:
//     0      'L'
//     1      flags: bit0 capture on, bit1 capturing, bit2 log ready,
//                   bit3 log filled up and stopped early, bit4 answering a G
//     2-3    log number                 uint16, counts up from power-up
//     4-5    samples in the log         uint16
//     6-9    millis() of the first      uint32
//     10     milliseconds between samples, nominal
//
//   A captured sample, in two halves of 20 bytes, sent only in answer to G.
//   While a G is being answered the packets above wait, so it has the radio.
//     0      'H'
//     1-2    sample number              uint16
//     3-4    time since the first       uint16, units of 2 ms
//     5      bits 0-3 AutoState, bit4 accelerometer trusted, bit5 motors driven
//     6-7    pitch                      int16, hundredths of a degree
//     8-9    pitch from the accelerometer alone, same units
//     10-11  roll                       same units
//     12-17  acceleration x, y, z       int16, thousandths of a g
//     18-19  heading change             int16, tenths of a degree
//
//     0      'I'
//     1-2    sample number              uint16
//     3-8    rotation rate x, y, z      int16, tenths of a deg/s, offset removed
//     9-10   channel A duty applied     int16
//     11-12  channel B duty applied     int16
//     13-14  pack voltage               uint16, millivolts
//     15     autopilot's left output    int8, power / 2
//     16     autopilot's right output   int8, power / 2
//     17-18  pack sense pin, raw        uint16
//     19     low byte of the log number, so halves of an older log are ignored
// ---------------------------------------------------------------------------

// Keeps the cause of the last restart available to System.resetReason().
STARTUP(System.enableFeature(FEATURE_RESET_INFO));

// Connects to the cloud in the background so new code can be flashed over
// Wi-Fi. setup() and loop() start straight away and don't wait for it.
SYSTEM_MODE(AUTOMATIC);

static const char *DEVICE_NAME = "Monarch";

static const unsigned long ESTIMATE_MS = 20;        // 50 Hz
static const unsigned long TELEMETRY_MS = 100;
static const unsigned long STATUS_MS = 1000;
static const unsigned long COMMAND_TIMEOUT_MS = 500;
static const unsigned long SENSOR_RETRY_MS = 2000;

// How long after the last change in motor output the robot is assumed to
// still be speeding up or slowing down, and the accelerometer left out.
static const unsigned long MOTORS_STEADY_MS = 600;

// One second of readings, averaged, is the gyro's resting offset.
static const int GYRO_ZERO_SAMPLES = 50;

// High-rate capture: how long a log runs on past the end of a run, and how
// much of Hold it keeps. A run can sit in Hold for a minute while someone
// measures where it stopped, and none of that needs 50 rows a second.
static const unsigned long CAPTURE_TAIL_MS = 2000;
static const unsigned long CAPTURE_HOLD_MS = 4000;

// --- motors -----------------------------------------------------------------

struct Motor {
    pin_t in1Pin;
    pin_t in2Pin;
    pin_t pwmPin;
    bool pwmArmed;      // true while pwmPin is running hardware PWM
};

static Motor motorLeft{D5, D6, MOSI, false};     // driver channel A
static Motor motorRight{D4, D3, MISO, false};    // driver channel B

// Low puts the whole driver to sleep, so nothing moves until this is high.
// Only connected to the driver in START_SWITCH wiring; with the switch on the
// standby pin (START_MOTION wiring) this must be left unconnected, or the
// pin and the switch would fight over the line.
static const pin_t PIN_STBY = D10;

// The start switch's output, in START_SWITCH wiring.
static const pin_t PIN_START_SWITCH = D2;

// --- pack voltage -------------------------------------------------------------
//
// A divider scales the pack down into the ADC's 0..3.3 V range:
//
//     pack + --- R_TOP --- A0 --- R_BOTTOM --- pack -
//
// with 100 nF from A0 to pack - to steady the reading against motor noise.
// 20k over 10k puts a full 8.4 V pack at 2.8 V on the pin. With nothing
// fitted, A0 floats and the reading means nothing.
static const pin_t PIN_PACK_SENSE = A0;
static const float PACK_R_TOP_OHMS = 20000.0f;
static const float PACK_R_BOTTOM_OHMS = 10000.0f;
static const float ADC_FULL_SCALE_VOLTS = 3.3f;
static const float ADC_MAX_COUNTS = 4095.0f;

// Long enough to hide PWM chatter and the millisecond spikes of a reversal,
// short enough that the sag while climbing (hundreds of ms) still shows.
static const float PACK_SMOOTHING_S = 0.3f;

// Resistor tolerance and ADC error, folded into one factor: set it to the
// multimeter's reading of the pack divided by the dashboard's.
static const float PACK_CALIBRATION = 1.0f;

static int packRawCounts = 0;       // latest ADC reading on the sense pin
static float packVolts = 0.0f;      // at the pack, smoothed

// Called at the estimator's 50 Hz.
static void samplePackVoltage() {
    static bool seeded = false;

    packRawCounts = analogRead(PIN_PACK_SENSE);
    float pinVolts = packRawCounts * ADC_FULL_SCALE_VOLTS / ADC_MAX_COUNTS;
    float volts = pinVolts * (PACK_R_TOP_OHMS + PACK_R_BOTTOM_OHMS) / PACK_R_BOTTOM_OHMS * PACK_CALIBRATION;

    // Exponential moving average, seeded with the first reading so the log
    // doesn't open on a ramp up from zero.
    const float dt = ESTIMATE_MS / 1000.0f;
    const float alpha = dt / (PACK_SMOOTHING_S + dt);
    packVolts = seeded ? packVolts + alpha * (volts - packVolts) : volts;
    seeded = true;
}

// How long a motor takes to go from stopped to full, so a flat-out reversal
// takes twice this. Easing through zero avoids the current spike of throwing
// a spinning motor straight into reverse.
static const unsigned long RAMP_FULL_SCALE_MS = 150;

// duty: 0..255. Fully on and fully off are plain digital levels. In between,
// the pin is armed for PWM once and then only its duty is rewritten: arming
// it again on every change would restart the timer and chop the output.
static void setPwm(Motor &m, int duty) {
    if (duty >= 255 || duty <= 0) {
        pinMode(m.pwmPin, OUTPUT);      // hands the pin back from PWM to GPIO
        digitalWrite(m.pwmPin, duty > 0 ? HIGH : LOW);
        m.pwmArmed = false;
        return;
    }

    if (!m.pwmArmed) {
        pinMode(m.pwmPin, OUTPUT);
        m.pwmArmed = true;
    }
    analogWrite(m.pwmPin, duty);
}

// Short brake: both inputs high shorts the motor's terminals through the
// driver, so it resists being turned. On a slope that beats coasting.
static void brake(Motor &m) {
    digitalWrite(m.in1Pin, HIGH);
    digitalWrite(m.in2Pin, HIGH);
    setPwm(m, 255);
}

// speed: -255..255. Sign sets direction, magnitude sets duty. 0 brakes.
static void drive(Motor &m, int speed) {
    speed = constrain(speed, -255, 255);

    if (speed == 0) {
        brake(m);
        return;
    }

    digitalWrite(m.in1Pin, speed > 0 ? HIGH : LOW);
    digitalWrite(m.in2Pin, speed > 0 ? LOW : HIGH);
    setPwm(m, abs(speed));
}

static void attach(Motor &m) {
    pinMode(m.in1Pin, OUTPUT);
    pinMode(m.in2Pin, OUTPUT);
    brake(m);
}

// Moves the applied speed toward its target by at most maxStep. Stopping is
// never eased: braking draws nothing from the supply, and a stop has to be
// immediate.
static float approach(float current, int target, float maxStep) {
    if (target == 0) return 0.0f;
    if (current < target) return min(current + maxStep, (float) target);
    return max(current - maxStep, (float) target);
}

// --- sensors ----------------------------------------------------------------

// Well under the usual 100 kHz. The ADXL345 breakout has no pull-ups of its
// own, and a bus with weak pull-ups rises slowly, which breaks register reads
// long before it breaks the address handshake.
static const uint32_t I2C_CLOCK_HZ = 20000;

static const uint8_t ADXL_ADDR = 0x53;              // SDO low
static const uint8_t ADXL_REG_DEVID = 0x00;
static const uint8_t ADXL_DEVID_EXPECTED = 0xE5;

static const uint8_t MPU_ADDR = 0x68;               // AD0 low
static const uint8_t MPU_REG_CONFIG = 0x1A;
static const uint8_t MPU_REG_GYRO_CONFIG = 0x1B;
static const uint8_t MPU_REG_ACCEL_CONFIG = 0x1C;
static const uint8_t MPU_REG_ACCEL_XOUT_H = 0x3B;
static const uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;
static const uint8_t MPU_REG_WHO_AM_I = 0x75;

// Ranges chosen in mpuBegin(): +/-4 g and +/-500 deg/s.
static const float MPU_COUNTS_PER_G = 8192.0f;
static const float MPU_COUNTS_PER_DPS = 65.5f;

static bool adxlPresent = false;
static bool mpuOk = false;

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

// The ADXL345 is only checked for; the estimator runs off the MPU6050, which
// has the gyro it needs.
static bool adxlDetect() {
    uint8_t id = 0;
    return readRegs(ADXL_ADDR, ADXL_REG_DEVID, &id, 1) && id == ADXL_DEVID_EXPECTED;
}

static bool mpuBegin() {
    uint8_t id = 0;
    if (!readRegs(MPU_ADDR, MPU_REG_WHO_AM_I, &id, 1)) return false;

    // The chip powers up asleep. 0x01 clears SLEEP and clocks it off the X
    // gyro, which the datasheet recommends over the internal oscillator.
    //
    // CONFIG 0x03 turns on the chip's own ~42 Hz low-pass filter. Sampled at
    // 50 Hz without it, chassis rattle far above 25 Hz would fold down into
    // the readings and look like slow motion.
    //
    // The wider ranges leave headroom: cresting the hill or taking a hit can
    // pass 250 deg/s and 2 g, and a gyro that clips mid-rotation leaves the
    // integrated angle wrong for good.
    return writeReg(MPU_ADDR, MPU_REG_PWR_MGMT_1, 0x01) &&
           writeReg(MPU_ADDR, MPU_REG_CONFIG, 0x03) &&
           writeReg(MPU_ADDR, MPU_REG_GYRO_CONFIG, 0x08) &&
           writeReg(MPU_ADDR, MPU_REG_ACCEL_CONFIG, 0x08);
}

// Restarts the bus from scratch and brings the sensors back up.
static void bringUpSensors() {
    Wire.end();
    Wire.setSpeed(I2C_CLOCK_HZ);
    Wire.begin();

    adxlPresent = adxlDetect();
    mpuOk = mpuBegin();
}

static int16_t bigEndian16(const uint8_t *p) {
    return (int16_t) ((p[0] << 8) | p[1]);
}

// accel in g, gyro in rad/s with nothing subtracted.
static bool readImu(Vec3 &accel, Vec3 &gyro) {
    uint8_t raw[14];    // accel xyz, temp, gyro xyz - a big-endian int16 each
    if (!readRegs(MPU_ADDR, MPU_REG_ACCEL_XOUT_H, raw, sizeof(raw))) return false;

    accel = Vec3{bigEndian16(raw + 0) / MPU_COUNTS_PER_G,
                 bigEndian16(raw + 2) / MPU_COUNTS_PER_G,
                 bigEndian16(raw + 4) / MPU_COUNTS_PER_G};
    gyro = Vec3{bigEndian16(raw + 8) / MPU_COUNTS_PER_DPS,
                bigEndian16(raw + 10) / MPU_COUNTS_PER_DPS,
                bigEndian16(raw + 12) / MPU_COUNTS_PER_DPS} * DEG_TO_RAD_F;
    return true;
}

// --- estimator --------------------------------------------------------------

static TiltEstimator estimator;
static Mounting mounting;

static Vec3 gyroBias{0.0f, 0.0f, 0.0f};         // rad/s
static Vec3 lastAccel{0.0f, 0.0f, 1.0f};        // g
static Vec3 lastGyro{0.0f, 0.0f, 0.0f};         // rad/s, offset removed

static int zeroSamplesLeft = 0;
static Vec3 zeroSum{0.0f, 0.0f, 0.0f};

static Autopilot autopilot;
static AutoParams params;
static float yawDeg = 0.0f;         // heading change since the run started, left positive

static void startGyroZero() {
    zeroSum = Vec3{0.0f, 0.0f, 0.0f};
    zeroSamplesLeft = GYRO_ZERO_SAMPLES;
}

// --- starting a run without the dashboard -----------------------------------
//
// See START_MOTION and START_SWITCH in Autopilot.h. Either way the robot is
// armed first - from the dashboard, or by itself at power-up - and then waits
// for its start signal with nobody touching the laptop.

static volatile bool armed = false;
static unsigned long lastMotorChange = 0;       // when the applied motor output last changed

// The start switch, in START_SWITCH. It connects the pin to 3V3 or to ground.
// The pull-down means a wire that has come off reads as "off", not as "go".
static const unsigned long SWITCH_DEBOUNCE_MS = 20;
static bool switchOn = false;
static bool switchSeenOff = false;      // armed has to see "off" before "on" counts
static bool runFromSwitch = false;

static void readSwitch(unsigned long now) {
    static bool lastRaw = false;
    static unsigned long rawSince = 0;

    bool raw = digitalRead(PIN_START_SWITCH) == HIGH;
    if (raw != lastRaw) {
        lastRaw = raw;
        rawSince = now;
    } else if (now - rawSince >= SWITCH_DEBOUNCE_MS) {
        switchOn = raw;
    }
}

// The launch detector, in START_MOTION. It first waits for the robot to sit
// still, so being carried to the ramp and set down doesn't count, and takes
// the acceleration it then sees as "at rest". Setting off shows up as a push
// along the robot's forward direction; a change twice that size in any
// direction counts too, in case forward was never calibrated.
static const unsigned long LAUNCH_SETTLE_MS = 300;
static const float STILL_ACCEL_G = 0.03f;
static const float STILL_GYRO = 5.0f * DEG_TO_RAD_F;
static const int LAUNCH_CONFIRM_SAMPLES = 2;
// Roughly how long the jolt takes to register. The run clock is wound back by
// this, so brake timing tuned from the dashboard carries over.
static const unsigned long LAUNCH_LATENCY_MS = 60;

static Vec3 launchBaseline{0.0f, 0.0f, 1.0f};
static Vec3 launchSmooth{0.0f, 0.0f, 1.0f};
static bool launchReady = false;
static bool launchDetected = false;
static unsigned long launchStillSince = 0;
static int launchHits = 0;

static void resetLaunchDetector(unsigned long now) {
    launchSmooth = lastAccel;
    launchReady = false;
    launchDetected = false;
    launchStillSince = now;
    launchHits = 0;
}

static void launchDetectorTick(unsigned long now, const Vec3 &accel, const Vec3 &gyro) {
    bool still = gyro.magnitude() < STILL_GYRO && (accel - launchSmooth).magnitude() < STILL_ACCEL_G;
    launchSmooth = launchSmooth + (accel - launchSmooth) * 0.2f;

    if (!launchReady) {
        if (!still) {
            launchStillSince = now;
        } else if (now - launchStillSince >= LAUNCH_SETTLE_MS) {
            launchReady = true;
            launchBaseline = launchSmooth;
            launchHits = 0;
        }
        return;
    }

    Vec3 change = accel - launchBaseline;
    float threshold = params[P_LAUNCH_MG] / 1000.0f;
    bool hit = (mounting.forwardKnown && change.dot(mounting.forward) > threshold) ||
               change.magnitude() > 2.0f * threshold;

    launchHits = hit ? launchHits + 1 : 0;
    if (launchHits >= LAUNCH_CONFIRM_SAMPLES) {
        launchDetected = true;
    } else if (still) {
        launchBaseline = launchBaseline + (accel - launchBaseline) * 0.02f;     // follow slow drift
    }
}

// The backstop for a launch the detector misses, in START_MOTION. Without it
// the robot would hold full power over the top and down the far side. Having
// sat nose-up on the ramp for a moment and then come level can only mean it
// climbed, so that goes straight to braking.
static const float ON_RAMP_MARGIN_DEG = 5.0f;
static const unsigned long ON_RAMP_CONFIRM_MS = 300;
static const unsigned long LEVELLED_CONFIRM_MS = 40;

static bool seenOnRamp = false;
static unsigned long noseUpSince = 0;
static unsigned long levelledSince = 0;

static bool climbedUnnoticed(unsigned long now, float pitchDeg) {
    float crest = (float) params[P_CREST_PITCH];

    if (pitchDeg > crest + ON_RAMP_MARGIN_DEG) {
        if (noseUpSince == 0) noseUpSince = now;
        if (now - noseUpSince >= ON_RAMP_CONFIRM_MS) seenOnRamp = true;
    } else {
        noseUpSince = 0;
    }

    if (!seenOnRamp || pitchDeg >= crest) {
        levelledSince = 0;
        return false;
    }

    if (levelledSince == 0) levelledSince = now;
    return now - levelledSince >= LEVELLED_CONFIRM_MS;
}

static void arm(unsigned long now) {
    armed = true;
    switchSeenOff = false;
    seenOnRamp = false;
    noseUpSince = 0;
    levelledSince = 0;
    resetLaunchDetector(now);
}

static void estimateTick(float dt, bool motorsSteady) {
    Vec3 accel;
    Vec3 gyroRaw;

    // A failed read marks the chip missing, so loop() brings the bus back up.
    if (!mpuOk || !readImu(accel, gyroRaw)) {
        mpuOk = false;
        return;
    }

    lastAccel = accel;

    if (zeroSamplesLeft > 0) {
        zeroSum = zeroSum + gyroRaw;
        if (--zeroSamplesLeft == 0) gyroBias = zeroSum * (1.0f / GYRO_ZERO_SAMPLES);

        // Zeroing assumes the robot is still, so the accelerometer is the
        // whole truth for now. Integrating a not-yet-zeroed gyro would only
        // walk the estimate away from it.
        lastGyro = Vec3{0.0f, 0.0f, 0.0f};
        estimator.reset(accel);
        return;
    }

    lastGyro = gyroRaw - gyroBias;
    estimator.update(lastGyro, accel, dt, motorsSteady);

    // Turning about the robot's own up direction is a change of heading. Only
    // tracked during a run, where the autopilot steers against it.
    if (autopilot.active()) yawDeg += lastGyro.dot(mounting.up) * dt * RAD_TO_DEG_F;

    if (armed && !autopilot.active() && params[P_START_MODE] == START_MOTION) {
        launchDetectorTick(millis(), accel, lastGyro);
    }
}

// The mounting directions survive power-off in the Photon's emulated EEPROM.
struct SavedMounting {
    uint32_t magic;
    float up[3];
    float forward[3];
    uint8_t forwardKnown;
};

static const uint32_t MOUNTING_MAGIC = 0x4D4F4E31;      // "MON1"
static const int MOUNTING_EEPROM_ADDR = 0;

static void loadMounting() {
    SavedMounting saved;
    EEPROM.get(MOUNTING_EEPROM_ADDR, saved);
    if (saved.magic != MOUNTING_MAGIC) return;      // never calibrated: keep the defaults

    mounting.up = Vec3{saved.up[0], saved.up[1], saved.up[2]}.normalized();
    mounting.forward = Vec3{saved.forward[0], saved.forward[1], saved.forward[2]}.normalized();
    mounting.forwardKnown = saved.forwardKnown != 0;
}

static void saveMounting() {
    SavedMounting saved{MOUNTING_MAGIC,
                        {mounting.up.x, mounting.up.y, mounting.up.z},
                        {mounting.forward.x, mounting.forward.y, mounting.forward.z},
                        (uint8_t) (mounting.forwardKnown ? 1 : 0)};
    EEPROM.put(MOUNTING_EEPROM_ADDR, saved);
}

// The autopilot's parameters are kept the same way, so a run uses the last
// values the dashboard sent even after a power cycle.
struct SavedParams {
    uint32_t magic;
    int16_t values[PARAM_COUNT];
};

// The count is part of the magic, so adding a parameter discards an old save
// rather than reading it into the wrong slots.
static const uint32_t PARAMS_MAGIC = 0x50415200 + PARAM_COUNT;      // "PAR" + count
static const int PARAMS_EEPROM_ADDR = 64;
static const unsigned long PARAMS_SAVE_DELAY_MS = 2000;

static volatile bool paramsDirty = false;
static volatile unsigned long paramsChangedAt = 0;
static volatile uint8_t paramsReceived = 0;

static void loadParams() {
    SavedParams saved;
    EEPROM.get(PARAMS_EEPROM_ADDR, saved);
    if (saved.magic != PARAMS_MAGIC) return;

    for (int i = 0; i < PARAM_COUNT; i++) params.set(i, saved.values[i]);
}

// Writes once the dashboard has gone quiet, and never mid-run: a flash write
// stalls the processor for a moment.
static void saveParamsWhenSettled(unsigned long now) {
    if (!paramsDirty || autopilot.active() || now - paramsChangedAt < PARAMS_SAVE_DELAY_MS) return;
    paramsDirty = false;

    SavedParams saved;
    saved.magic = PARAMS_MAGIC;
    for (int i = 0; i < PARAM_COUNT; i++) saved.values[i] = params.values[i];
    EEPROM.put(PARAMS_EEPROM_ADDR, saved);
}

// --- Bluetooth --------------------------------------------------------------

static const BleUuid SERVICE_UUID("6E400001-B5A3-F393-E0A9-E50E24DCCA9E");
static const BleUuid RX_UUID("6E400002-B5A3-F393-E0A9-E50E24DCCA9E");
static const BleUuid TX_UUID("6E400003-B5A3-F393-E0A9-E50E24DCCA9E");

static void onCommandBytes(const uint8_t *data, size_t len, const BlePeerDevice &peer, void *context);

static BleCharacteristic txCharacteristic("tx", BleCharacteristicProperty::NOTIFY,
                                          TX_UUID, SERVICE_UUID);
static BleCharacteristic rxCharacteristic("rx", BleCharacteristicProperty::WRITE_WO_RSP,
                                          RX_UUID, SERVICE_UUID, onCommandBytes, nullptr);

// Written from the Bluetooth thread, read from loop(). Each is a single word,
// so a read can never see half an update.
static volatile int commandLeft = 0;
static volatile int commandRight = 0;
static volatile unsigned long commandAtMs = 0;
static volatile bool commandSeen = false;

// Calibration requests are only flagged here and carried out in loop(), which
// owns the estimator and the EEPROM.
static volatile bool requestZero = false;
static volatile bool requestLevel = false;
static volatile bool requestNoseUp = false;

// The run is started and ended in loop() too. A pending end carries its reason.
static volatile bool requestAutoStart = false;
static volatile uint8_t requestAutoEnd = (uint8_t) EndReason::None;
static volatile bool requestArm = false;
static volatile bool requestDisarm = false;

// High-rate capture, likewise carried out in loop().
static volatile bool captureWanted = false;
static volatile bool requestCaptureBegin = false;
static volatile bool requestCaptureFinish = false;
static volatile bool syncRequested = false;
static volatile int syncRequestFirst = 0;
static volatile int syncRequestCount = 0;
static volatile int syncRequestPaceMs = 0;

static void handleCommandLine(const char *line) {
    int left = 0;
    int right = 0;
    int id = 0;
    long value = 0;
    int first = 0;
    int count = 0;
    int pace = 0;

    if (strcmp(line, "Z") == 0) { requestZero = true; return; }
    if (strcmp(line, "L") == 0) { requestLevel = true; return; }
    if (strcmp(line, "U") == 0) { requestNoseUp = true; return; }
    if (strcmp(line, "A") == 0) { requestAutoStart = true; return; }
    if (strcmp(line, "ARM") == 0) { requestArm = true; return; }
    if (strcmp(line, "HS") == 0) { requestCaptureBegin = true; return; }
    if (strcmp(line, "HE") == 0) { requestCaptureFinish = true; return; }

    if (sscanf(line, "H,%d", &id) == 1) {
        captureWanted = id != 0;
        return;
    }

    // Recognised before the catch-all stop below, so fetching a log never
    // ends a run or disarms the robot.
    if (sscanf(line, "G,%d,%d,%d", &first, &count, &pace) == 3) {
        syncRequestFirst = first;
        syncRequestCount = count;
        syncRequestPaceMs = pace;
        syncRequested = true;
        return;
    }

    if (sscanf(line, "P,%d,%ld", &id, &value) == 2) {
        if (params.set(id, value)) {
            paramsDirty = true;
            paramsChangedAt = millis();
        }
        paramsReceived++;
        return;
    }

    if (sscanf(line, "D,%d,%d", &left, &right) == 2) {
        // The dashboard's idle zeros must not cancel a run it just started.
        if (autopilot.active() && left == 0 && right == 0) return;

        // Driving by hand takes over from a run, and from waiting for one.
        if (autopilot.active()) requestAutoEnd = (uint8_t) EndReason::ManualTakeover;
        if (armed && (left != 0 || right != 0)) requestDisarm = true;
        commandLeft = constrain(left, -255, 255);
        commandRight = constrain(right, -255, 255);
    } else {
        // "S", or anything unreadable: stopping is the safe reading of both.
        if (autopilot.active()) requestAutoEnd = (uint8_t) EndReason::StopCommand;
        if (armed) requestDisarm = true;
        commandLeft = 0;
        commandRight = 0;
    }

    commandAtMs = millis();
    commandSeen = true;
}

// Bytes arrive in whatever chunks the radio delivers, so collect them into a
// line and act on the newline.
static void onCommandBytes(const uint8_t *data, size_t len, const BlePeerDevice &peer, void *context) {
    static char line[32];
    static size_t lineLen = 0;

    for (size_t i = 0; i < len; i++) {
        char c = (char) data[i];

        if (c == '\n' || c == '\r') {
            if (lineLen > 0) {
                line[lineLen] = '\0';
                handleCommandLine(line);
                lineLen = 0;
            }
        } else if (lineLen < sizeof(line) - 1) {
            line[lineLen++] = c;
        }
    }
}

static int16_t toInt16(float value) {
    return (int16_t) constrain(lroundf(value), -32768L, 32767L);
}

static void putInt16(uint8_t *p, float value) {
    int16_t v = toInt16(value);
    p[0] = (uint8_t) (v & 0xFF);
    p[1] = (uint8_t) ((v >> 8) & 0xFF);
}

static void putVec3(uint8_t *p, const Vec3 &v, float scale) {
    putInt16(p + 0, v.x * scale);
    putInt16(p + 2, v.y * scale);
    putInt16(p + 4, v.z * scale);
}

static void sendEstimate(bool motorsDriven) {
    uint8_t packet[20] = {0};

    packet[0] = 'E';
    packet[1] = (adxlPresent ? 0x01 : 0) |
                (mpuOk ? 0x02 : 0) |
                (motorsDriven ? 0x04 : 0) |
                (estimator.accelTrusted() ? 0x08 : 0) |
                (mounting.forwardKnown ? 0x10 : 0) |
                (zeroSamplesLeft > 0 ? 0x20 : 0);

    putVec3(packet + 2, lastAccel, 1000.0f);
    putVec3(packet + 8, lastGyro, RAD_TO_DEG_F * 10.0f);
    putInt16(packet + 14, mounting.pitch(estimator.up()) * 100.0f);
    putInt16(packet + 16, mounting.roll(estimator.up()) * 100.0f);
    putInt16(packet + 18, mounting.pitch(lastAccel.normalized()) * 100.0f);

    txCharacteristic.setValue(packet, sizeof(packet));
}

// Why the Photon last restarted and how long it has been up. A restart
// mid-drive shows up on the dashboard as the uptime starting over, with the
// cause beside it: power loss and a software crash report differently.
static void sendStatus() {
    uint8_t packet[10] = {0};
    uint32_t reasonData = System.resetReasonData();
    uint32_t uptimeSeconds = System.uptime();

    packet[0] = 'R';
    packet[1] = (uint8_t) System.resetReason();
    memcpy(packet + 2, &reasonData, sizeof(reasonData));        // little-endian MCU
    memcpy(packet + 6, &uptimeSeconds, sizeof(uptimeSeconds));

    txCharacteristic.setValue(packet, sizeof(packet));
}

// The dashboard needs these to split the raw rotation rates into pitch rate
// and yaw rate itself.
static void sendCalibration() {
    uint8_t packet[13] = {0};

    packet[0] = 'C';
    putVec3(packet + 1, mounting.up, 10000.0f);
    putVec3(packet + 7, mounting.forward, 10000.0f);

    txCharacteristic.setValue(packet, sizeof(packet));
}

static AutoState reportedState() {
    return autopilot.active() ? autopilot.state() : armed ? AutoState::Armed : AutoState::Idle;
}

static void sendAutopilot(unsigned long now, int left, int right) {
    uint8_t packet[13] = {0};

    packet[0] = 'A';
    packet[1] = (uint8_t) reportedState();
    packet[2] = (uint8_t) autopilot.lastEnd();

    // uint16 hundredths of a second tops out near eleven minutes.
    uint16_t stateTime = (uint16_t) min(autopilot.msInState(now) / 10UL, 65535UL);
    uint16_t runTime = (uint16_t) min(autopilot.msInRun(now) / 10UL, 65535UL);
    memcpy(packet + 3, &stateTime, sizeof(stateTime));
    memcpy(packet + 5, &runTime, sizeof(runTime));

    putInt16(packet + 7, yawDeg * 10.0f);
    packet[9] = (uint8_t) (int8_t) (left / 2);
    packet[10] = (uint8_t) (int8_t) (right / 2);
    packet[11] = paramsReceived;
    packet[12] = (switchOn ? 0x01 : 0) | (launchReady ? 0x02 : 0);

    txCharacteristic.setValue(packet, sizeof(packet));
}

static void sendMotors(unsigned long now, int channelA, int channelB) {
    uint8_t packet[13] = {0};
    uint32_t robotMs = (uint32_t) now;
    uint16_t millivolts = (uint16_t) constrain(lroundf(packVolts * 1000.0f), 0L, 65535L);
    uint16_t raw = (uint16_t) packRawCounts;

    packet[0] = 'M';
    memcpy(packet + 1, &robotMs, sizeof(robotMs));
    putInt16(packet + 5, channelA);
    putInt16(packet + 7, channelB);
    memcpy(packet + 9, &millivolts, sizeof(millivolts));
    memcpy(packet + 11, &raw, sizeof(raw));

    txCharacteristic.setValue(packet, sizeof(packet));
}

// --- high-rate capture ------------------------------------------------------

static CaptureLog capture;
static unsigned long captureEndAt = 0;      // when an ended run's log closes; 0 while it runs

// The G request being answered: samples syncNext .. syncEnd-1.
static int syncNext = 0;
static int syncEnd = 0;
static unsigned long syncPaceMs = 10;
static unsigned long lastSyncSample = 0;
static bool syncSecondHalf = false;

static void sendCaptureStatus() {
    uint8_t packet[11] = {0};
    uint16_t id = capture.id();
    uint16_t count = (uint16_t) capture.count();
    uint32_t firstMs = capture.count() > 0 ? capture.at(0).ms : 0;

    packet[0] = 'L';
    packet[1] = (capture.enabled() ? 0x01 : 0) |
                (capture.capturing() ? 0x02 : 0) |
                (capture.ready() ? 0x04 : 0) |
                (capture.truncated() ? 0x08 : 0) |
                (syncNext < syncEnd ? 0x10 : 0);
    memcpy(packet + 2, &id, sizeof(id));
    memcpy(packet + 4, &count, sizeof(count));
    memcpy(packet + 6, &firstMs, sizeof(firstMs));
    packet[10] = (uint8_t) ESTIMATE_MS;

    txCharacteristic.setValue(packet, sizeof(packet));
}

// One sample per estimator tick, in the same units as the live packets.
static void captureTick(unsigned long now, bool motorsDriven, int autoLeft, int autoRight,
                        int channelA, int channelB) {
    if (!capture.enabled() || !mpuOk) return;

    Vec3 up = estimator.up();
    CaptureSample s;
    s.ms = (uint32_t) now;
    s.pitchFused = toInt16(mounting.pitch(up) * 100.0f);
    s.pitchAccel = toInt16(mounting.pitch(lastAccel.normalized()) * 100.0f);
    s.roll = toInt16(mounting.roll(up) * 100.0f);
    s.accel[0] = toInt16(lastAccel.x * 1000.0f);
    s.accel[1] = toInt16(lastAccel.y * 1000.0f);
    s.accel[2] = toInt16(lastAccel.z * 1000.0f);
    s.gyro[0] = toInt16(lastGyro.x * RAD_TO_DEG_F * 10.0f);
    s.gyro[1] = toInt16(lastGyro.y * RAD_TO_DEG_F * 10.0f);
    s.gyro[2] = toInt16(lastGyro.z * RAD_TO_DEG_F * 10.0f);
    s.yaw = toInt16(yawDeg * 10.0f);
    s.motorA = (int16_t) channelA;
    s.motorB = (int16_t) channelB;
    s.packMv = (uint16_t) constrain(lroundf(packVolts * 1000.0f), 0L, 65535L);
    s.packRaw = (uint16_t) packRawCounts;
    s.autoLeft = (int8_t) (autoLeft / 2);
    s.autoRight = (int8_t) (autoRight / 2);
    s.stage = (uint8_t) reportedState();
    s.flags = (estimator.accelTrusted() ? CAPTURE_TRUSTED : 0) | (motorsDriven ? CAPTURE_DRIVEN : 0);
    capture.add(s);
}

static void beginCapture() {
    capture.begin();
    captureEndAt = 0;
    syncNext = syncEnd = 0;         // whatever was being fetched is gone
}

static void finishCapture() {
    if (!capture.capturing()) return;
    capture.finish();
    captureEndAt = 0;
    if (BLE.connected()) sendCaptureStatus();       // so the dashboard can fetch it at once
}

// A log opens as a run starts and closes a little after it ends, or once it
// has been holding for a while. HS and HE do the same by hand.
static void handleCapture(unsigned long now) {
    static bool wasRunning = false;
    bool running = autopilot.active();

    capture.enable(captureWanted);

    if (requestCaptureBegin) {
        requestCaptureBegin = false;
        beginCapture();
    }
    if (running && !wasRunning) beginCapture();
    if (!running && wasRunning && capture.capturing()) captureEndAt = now + CAPTURE_TAIL_MS;
    wasRunning = running;

    if (running && autopilot.state() == AutoState::Hold && autopilot.msInState(now) >= CAPTURE_HOLD_MS) {
        finishCapture();
    }
    if (captureEndAt != 0 && (long) (now - captureEndAt) >= 0) finishCapture();

    if (requestCaptureFinish) {
        requestCaptureFinish = false;
        finishCapture();
    }
}

static bool sendCaptureHalf(int i, bool second) {
    const CaptureSample &s = capture.at(i);
    uint8_t packet[20] = {0};
    uint16_t index = (uint16_t) i;

    packet[0] = second ? 'I' : 'H';
    memcpy(packet + 1, &index, sizeof(index));

    if (!second) {
        uint16_t elapsed = (uint16_t) min((s.ms - capture.at(0).ms) / 2UL, 65535UL);
        memcpy(packet + 3, &elapsed, sizeof(elapsed));
        packet[5] = (s.stage & 0x0F) | ((s.flags & CAPTURE_TRUSTED) ? 0x10 : 0) |
                    ((s.flags & CAPTURE_DRIVEN) ? 0x20 : 0);
        memcpy(packet + 6, &s.pitchFused, 2);
        memcpy(packet + 8, &s.pitchAccel, 2);
        memcpy(packet + 10, &s.roll, 2);
        memcpy(packet + 12, s.accel, 6);
        memcpy(packet + 18, &s.yaw, 2);
    } else {
        memcpy(packet + 3, s.gyro, 6);
        memcpy(packet + 9, &s.motorA, 2);
        memcpy(packet + 11, &s.motorB, 2);
        memcpy(packet + 13, &s.packMv, 2);
        packet[15] = (uint8_t) s.autoLeft;
        packet[16] = (uint8_t) s.autoRight;
        memcpy(packet + 17, &s.packRaw, 2);
        packet[19] = (uint8_t) (capture.id() & 0xFF);
    }

    return txCharacteristic.setValue(packet, sizeof(packet)) >= 0;
}

// Answers G one sample at a time, at the pace the dashboard asked for. A half
// the radio wouldn't take is offered again on the next pass, and anything lost
// on the way the dashboard simply asks for again. Returns whether a request is
// being answered, so the live feed can wait.
static bool serveSync(unsigned long now) {
    if (syncRequested) {
        syncRequested = false;
        int first = syncRequestFirst;
        int count = syncRequestCount;
        if (capture.ready() && first >= 0 && count > 0 && first < capture.count()) {
            syncNext = first;
            syncEnd = min(first + count, capture.count());
            syncPaceMs = (unsigned long) constrain(syncRequestPaceMs, 2, 100);
            syncSecondHalf = false;
            lastSyncSample = now - syncPaceMs;
        } else if (BLE.connected()) {
            sendCaptureStatus();        // nothing to send: say why
        }
    }

    if (syncNext >= syncEnd) return false;
    if (!BLE.connected() || !capture.ready()) {
        syncNext = syncEnd = 0;
        return false;
    }
    if (now - lastSyncSample < syncPaceMs) return true;

    if (!syncSecondHalf) {
        if (!sendCaptureHalf(syncNext, false)) return true;
        syncSecondHalf = true;
    }
    if (!sendCaptureHalf(syncNext, true)) return true;
    syncSecondHalf = false;
    syncNext++;
    lastSyncSample = now;

    if (syncNext >= syncEnd) sendCaptureStatus();      // this range is done
    return true;
}

// Turns the autopilot's left and right into the two driver channels, the same
// way the dashboard does for manual driving: trim first, then the side swap,
// then each channel's direction.
static void mapToMotors(int left, int right, int &channelA, int &channelB) {
    float a = left * params[P_TRIM_LEFT] / 1000.0f;
    float b = right * params[P_TRIM_RIGHT] / 1000.0f;

    if (params.flag(FLAG_SWAP_SIDES)) {
        float swap = a;
        a = b;
        b = swap;
    }
    if (params.flag(FLAG_INVERT_LEFT)) a = -a;
    if (params.flag(FLAG_INVERT_RIGHT)) b = -b;

    channelA = (int) lroundf(a);
    channelB = (int) lroundf(b);
}

static void handleCalibrationRequests() {
    if (requestZero) {
        requestZero = false;
        startGyroZero();
    }

    if (requestLevel) {
        requestLevel = false;
        mounting.setLevel(estimator.up());
        saveMounting();
        startGyroZero();        // level means still, so take the chance
    }

    if (requestNoseUp) {
        requestNoseUp = false;
        if (mounting.setNoseUp(estimator.up())) saveMounting();
    }
}

// A run started on the robot itself is meant to work with no laptop in reach,
// so it is never ended by the Bluetooth link being down.
static bool runStartedOnRobot = false;

// startedAt is when the run is taken to have begun, which for a launch felt
// by the accelerometer is a little before now.
static void beginRun(unsigned long startedAt, bool onRobot, bool fromSwitch) {
    yawDeg = 0.0f;
    runStartedOnRobot = onRobot;
    runFromSwitch = fromSwitch;
    // No time limit either: on the robot, the switch is what ends it.
    autopilot.start(startedAt, onRobot);

    // The robot is about to accelerate even if the motor outputs don't change
    // (in START_MOTION they were already at full), so keep the accelerometer
    // out of the estimate for the usual spell.
    lastMotorChange = millis();

    if (!mpuOk) autopilot.stop(EndReason::SensorMissing);
}

// Arming, the start signals, and stopping. A stop is handled last, so that
// when a start and a stop arrive in the same pass the stop wins.
static void handleRunRequests(unsigned long now) {
    static bool armAtBootPending = true;
    int mode = params[P_START_MODE];

    // Arming at power-up waits for the gyro to finish zeroing.
    if (armAtBootPending && mpuOk && zeroSamplesLeft == 0) {
        armAtBootPending = false;
        if (params.flag(FLAG_ARM_ON_BOOT) && mode != START_DASHBOARD) arm(now);
    }

    if (requestArm) {
        requestArm = false;
        if (mode != START_DASHBOARD) arm(now);
    }

    if (requestAutoStart) {
        requestAutoStart = false;
        armed = false;
        beginRun(now, false, false);
    }

    if (armed && !autopilot.active()) {
        if (mode == START_SWITCH) {
            // Stays armed through the run, so switching off and on again
            // gives another go without the dashboard.
            if (!switchOn) {
                switchSeenOff = true;
            } else if (switchSeenOff) {
                switchSeenOff = false;
                beginRun(now, true, true);
            }
        } else if (mode == START_MOTION) {
            if (launchDetected) {
                armed = false;
                beginRun(now - LAUNCH_LATENCY_MS, true, false);
            } else if (climbedUnnoticed(now, mounting.pitch(estimator.up()))) {
                // Already at the top: begin as if the braking time had just
                // arrived, so the run opens with Brake and then Hold.
                armed = false;
                beginRun(now - (unsigned long) params[P_BRAKE_START_MS], true, false);
            }
        } else {
            armed = false;      // the mode was changed away while waiting
        }
    }

    // In START_SWITCH the switch is also the off switch.
    if (autopilot.active() && runFromSwitch && mode == START_SWITCH && !switchOn) {
        autopilot.stop(EndReason::SwitchOff);
        switchSeenOff = true;
    }

    if (requestDisarm) {
        requestDisarm = false;
        armed = false;
    }

    if (requestAutoEnd != (uint8_t) EndReason::None) {
        autopilot.stop((EndReason) (uint8_t) requestAutoEnd);
        requestAutoEnd = (uint8_t) EndReason::None;
        armed = false;
    }
}

void setup() {
    attach(motorLeft);
    attach(motorRight);

    // Asleep until loop() has read the start mode and decided.
    pinMode(PIN_STBY, OUTPUT);
    digitalWrite(PIN_STBY, LOW);
    pinMode(PIN_START_SWITCH, INPUT_PULLDOWN);

    Serial.begin(115200);

    loadMounting();
    loadParams();
    capture.allocate();
    bringUpSensors();
    startGyroZero();            // the robot is expected to be still at power-up

    BLE.on();
    BLE.setDeviceName(DEVICE_NAME);
    BLE.addCharacteristic(txCharacteristic);
    BLE.addCharacteristic(rxCharacteristic);

    // The 128-bit service UUID nearly fills the advertising packet, so the
    // name rides in the scan response instead.
    BleAdvertisingData advertising;
    advertising.appendServiceUUID(SERVICE_UUID);

    BleAdvertisingData scanResponse;
    scanResponse.appendLocalName(DEVICE_NAME);

    BLE.advertise(&advertising, &scanResponse);
}

void loop() {
    static unsigned long lastEstimate = 0;
    static unsigned long lastTelemetry = 0;
    static unsigned long lastStatus = 0;
    static unsigned long lastSensorRetry = 0;
    static unsigned long lastRamp = 0;
    static float rampLeft = 0.0f;
    static float rampRight = 0.0f;
    static int appliedLeft = 0;
    static int appliedRight = 0;
    static int standbyLevel = -1;

    unsigned long now = millis();

    // In START_MOTION the switch owns the driver's standby pin, so this pin
    // stays low. If standby were still wired here by mistake, the motors would
    // then simply never run - far better than arming and launching at once.
    int wantStandby = params[P_START_MODE] == START_MOTION ? LOW : HIGH;
    if (wantStandby != standbyLevel) {
        standbyLevel = wantStandby;
        digitalWrite(PIN_STBY, wantStandby);
    }

    readSwitch(now);
    handleRunRequests(now);

    // A run can't continue blind, and by default not out of reach either.
    if (autopilot.active()) {
        if (!mpuOk) {
            autopilot.stop(EndReason::SensorMissing);
        } else if (!runStartedOnRobot && params.flag(FLAG_STOP_ON_LINK_LOSS) && !BLE.connected()) {
            autopilot.stop(EndReason::LinkLost);
        }
    }

    int targetLeft = 0;
    int targetRight = 0;
    int autoLeft = 0;
    int autoRight = 0;

    if (autopilot.active()) {
        autopilot.update(now, mounting.pitch(estimator.up()), yawDeg, params, autoLeft, autoRight);
        mapToMotors(autoLeft, autoRight, targetLeft, targetRight);
    } else if (armed && params[P_START_MODE] == START_MOTION) {
        // Full power asked for the whole time. The sleeping driver ignores it
        // until the switch wakes it, and then the robot goes at once.
        autoLeft = params[P_CLIMB_POWER];
        autoRight = params[P_CLIMB_POWER];
        mapToMotors(autoLeft, autoRight, targetLeft, targetRight);
    } else if (armed) {
        // START_SWITCH: the driver is awake and the targets are zero, which
        // brakes the motors and holds the robot where it sits on the ramp.
    } else {
        // Dead-man: only a fresh command from a connected dashboard keeps the
        // motors turning.
        bool live = BLE.connected() && commandSeen && now - commandAtMs < COMMAND_TIMEOUT_MS;
        targetLeft = live ? commandLeft : 0;
        targetRight = live ? commandRight : 0;
    }

    float maxStep = 255.0f * (now - lastRamp) / RAMP_FULL_SCALE_MS;
    lastRamp = now;
    rampLeft = approach(rampLeft, targetLeft, maxStep);
    rampRight = approach(rampRight, targetRight, maxStep);

    // Only touch the driver when the applied speed actually changes.
    int left = (int) lroundf(rampLeft);
    int right = (int) lroundf(rampRight);
    if (left != appliedLeft || right != appliedRight) {
        if (left != appliedLeft) drive(motorLeft, left);
        if (right != appliedRight) drive(motorRight, right);
        appliedLeft = left;
        appliedRight = right;
        lastMotorChange = now;
    }
    bool motorsDriven = left != 0 || right != 0;
    bool motorsSteady = now - lastMotorChange >= MOTORS_STEADY_MS;

    handleCapture(now);

    if (now - lastEstimate >= ESTIMATE_MS) {
        // Capped, so one long stall can't turn into one enormous rotation step.
        float dt = min((now - lastEstimate) / 1000.0f, 0.1f);
        lastEstimate = now;
        estimateTick(dt, motorsSteady);
        samplePackVoltage();
        captureTick(now, motorsDriven, autoLeft, autoRight, appliedLeft, appliedRight);
    }

    handleCalibrationRequests();

    if (!mpuOk && now - lastSensorRetry >= SENSOR_RETRY_MS) {
        lastSensorRetry = now;
        bringUpSensors();
    }

    saveParamsWhenSettled(now);

    // A log being fetched has the radio to itself until it is done.
    bool syncing = serveSync(now);

    if (!syncing && BLE.connected() && now - lastTelemetry >= TELEMETRY_MS) {
        lastTelemetry = now;
        sendEstimate(motorsDriven);
        sendAutopilot(now, autoLeft, autoRight);
        sendMotors(now, appliedLeft, appliedRight);
    }

    if (!syncing && BLE.connected() && now - lastStatus >= STATUS_MS) {
        lastStatus = now;
        sendStatus();
        sendCalibration();
        sendCaptureStatus();
    }

    delay(2);
}
