#include "Particle.h"

// ---------------------------------------------------------------------------
// Photon 2 motor bench test - temporary, not part of the robot.
//
// Sweeps one motor at a time from stopped up to full speed and back down,
// forward and then reverse, printing the duty as it goes. Driver is the
// SparkFun TB6612FNG.
//
//   particle flash --local              (from this folder, Photon on USB)
//   particle serial monitor --follow
//
// The PWM inputs are on MOSI and MISO because only D1, A2, A5, MOSI and MISO
// have hardware PWM on the Photon 2.
//
// What to watch for: the duty at which the motor starts to turn, and whether
// it keeps getting stronger all the way to 255 or fades out before it.
// ---------------------------------------------------------------------------

// Connects to the cloud in the background so new code can be flashed over
// Wi-Fi. setup() and loop() start straight away and don't wait for it.
SYSTEM_MODE(AUTOMATIC);

struct Motor {
    const char *name;
    pin_t in1Pin;
    pin_t in2Pin;
    pin_t pwmPin;
};

static const Motor MOTOR_A{"A", D5, D6, MOSI};
static const Motor MOTOR_B{"B", D4, D3, MISO};

// Low puts the whole driver to sleep, so nothing moves until this is high.
static const pin_t PIN_STBY = D10;

// Which motors the sweep exercises.
static const bool RUN_MOTOR_A = true;
static const bool RUN_MOTOR_B = false;

static const int DUTY_STEP = 5;
static const unsigned long STEP_MS = 40;        // 0 -> 255 in about two seconds
static const unsigned long TOP_HOLD_MS = 1500;
static const unsigned long REST_MS = 700;
static const int PRINT_EVERY = 50;              // duty counts between print lines

// duty: 0..255. Fully on and fully off are plain digital levels, so the two
// ends of the sweep don't depend on the PWM hardware. pinMode() first hands
// the pin back to GPIO if it was last used for PWM.
static void setPwm(const Motor &m, int duty) {
    pinMode(m.pwmPin, OUTPUT);

    if (duty >= 255) {
        digitalWrite(m.pwmPin, HIGH);
    } else if (duty <= 0) {
        digitalWrite(m.pwmPin, LOW);
    } else {
        analogWrite(m.pwmPin, duty);
    }
}

static void setDirection(const Motor &m, bool forward) {
    digitalWrite(m.in1Pin, forward ? HIGH : LOW);
    digitalWrite(m.in2Pin, forward ? LOW : HIGH);
}

// Both inputs low with PWM at 0 leaves the motor undriven.
static void release(const Motor &m) {
    digitalWrite(m.in1Pin, LOW);
    digitalWrite(m.in2Pin, LOW);
    setPwm(m, 0);
}

static void attach(const Motor &m) {
    pinMode(m.in1Pin, OUTPUT);
    pinMode(m.in2Pin, OUTPUT);
    release(m);
}

static void setDutyAndReport(const Motor &m, const char *direction, int duty) {
    setPwm(m, duty);
    if (duty % PRINT_EVERY == 0 || duty == 255) {
        Serial.printlnf("motor %s %s: duty %d", m.name, direction, duty);
    }
}

static void sweep(const Motor &m, bool forward) {
    const char *direction = forward ? "fwd" : "rev";
    setDirection(m, forward);

    for (int duty = 0; duty <= 255; duty += DUTY_STEP) {
        setDutyAndReport(m, direction, duty);
        delay(STEP_MS);
    }

    Serial.printlnf("motor %s %s: holding 255", m.name, direction);
    delay(TOP_HOLD_MS);

    for (int duty = 255; duty >= 0; duty -= DUTY_STEP) {
        setDutyAndReport(m, direction, duty);
        delay(STEP_MS);
    }

    release(m);
    delay(REST_MS);
}

void setup() {
    attach(MOTOR_A);
    attach(MOTOR_B);

    // Wake the driver only once both motors are set to undriven.
    pinMode(PIN_STBY, OUTPUT);
    digitalWrite(PIN_STBY, HIGH);

    Serial.begin(115200);
    delay(1000);
    Serial.println("motor sweep test");
}

void loop() {
    if (RUN_MOTOR_A) {
        sweep(MOTOR_A, true);
        sweep(MOTOR_A, false);
    }

    if (RUN_MOTOR_B) {
        sweep(MOTOR_B, true);
        sweep(MOTOR_B, false);
    }
}
