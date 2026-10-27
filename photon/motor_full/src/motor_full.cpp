#include "Particle.h"

// ---------------------------------------------------------------------------
// Photon 2 full-throttle test - temporary, not part of the robot.
//
// Runs both motors flat out for a few seconds, brakes, rests, and repeats, so
// top speed can be judged. The PWM pins are held as plain highs rather than
// driven with analogWrite(), so this is a true 100%: the motors see the full
// supply for the whole burst. Driver is the SparkFun TB6612FNG.
//
//   particle flash p2348                (from this folder, over Wi-Fi)
//
// The rest between bursts is there to catch the robot. If it spins on the
// spot instead of driving straight, one motor is mounted mirrored: flip its
// *_FORWARD flag below.
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

static const bool A_FORWARD = true;
static const bool B_FORWARD = true;

static const unsigned long RUN_MS = 4000;
static const unsigned long REST_MS = 3000;

static void fullThrottle(const Motor &m, bool forward) {
    digitalWrite(m.in1Pin, forward ? HIGH : LOW);
    digitalWrite(m.in2Pin, forward ? LOW : HIGH);
    digitalWrite(m.pwmPin, HIGH);
}

// Short brake: both inputs high shorts the motor's terminals through the
// driver, which stops it far quicker than letting it coast.
static void brake(const Motor &m) {
    digitalWrite(m.in1Pin, HIGH);
    digitalWrite(m.in2Pin, HIGH);
    digitalWrite(m.pwmPin, HIGH);
}

static void attach(const Motor &m) {
    pinMode(m.in1Pin, OUTPUT);
    pinMode(m.in2Pin, OUTPUT);
    pinMode(m.pwmPin, OUTPUT);
    brake(m);
}

void setup() {
    attach(MOTOR_A);
    attach(MOTOR_B);

    pinMode(PIN_STBY, OUTPUT);
    digitalWrite(PIN_STBY, HIGH);

    Serial.begin(115200);
    delay(REST_MS);     // time to set the robot down after a reset
}

void loop() {
    Serial.println("both motors: full throttle");
    fullThrottle(MOTOR_A, A_FORWARD);
    fullThrottle(MOTOR_B, B_FORWARD);
    delay(RUN_MS);

    Serial.println("both motors: brake");
    brake(MOTOR_A);
    brake(MOTOR_B);
    delay(REST_MS);
}
