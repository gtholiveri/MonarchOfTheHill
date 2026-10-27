#include "Particle.h"

// ---------------------------------------------------------------------------
// Photon 2 motor probe - temporary, stands in for a multimeter.
//
// Reads the driver's two supply pins with the Photon's ADC, first with the
// driver asleep and then with each motor held at full speed, and prints the
// voltages. A supply that reads fine asleep and sags under load is a bad
// power path; one that holds up while the motor still doesn't turn points at
// the driver. Nothing uses PWM, so it runs silently.
//
//   particle flash --local              (from this folder, Photon on USB)
//   particle serial monitor --follow
//
// Wiring for the two measurements:
//
//   driver VM  --[ R ]--+--[ R ]-- GND        two equal resistors, 1k - 10k
//                       |
//                       A0
//
//   driver VCC --------- A1
//
// The divider halves VM so it fits the ADC's 0 - 3.3 V range. VM must never
// go straight to a Photon pin.
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

static const pin_t PIN_STBY = D10;
static const pin_t PIN_VM_SENSE = A0;
static const pin_t PIN_VCC_SENSE = A1;

static const float ADC_REF_VOLTS = 3.3f;
static const float ADC_FULL_SCALE = 4095.0f;    // 12-bit
static const float VM_DIVIDER = 2.0f;           // two equal resistors

static const int SAMPLES_PER_PHASE = 4;
static const unsigned long SAMPLE_MS = 500;

static float readVolts(pin_t pin) {
    return analogRead(pin) * ADC_REF_VOLTS / ADC_FULL_SCALE;
}

static void fullForward(const Motor &m) {
    digitalWrite(m.in1Pin, HIGH);
    digitalWrite(m.in2Pin, LOW);
    digitalWrite(m.pwmPin, HIGH);
}

static void release(const Motor &m) {
    digitalWrite(m.in1Pin, LOW);
    digitalWrite(m.in2Pin, LOW);
    digitalWrite(m.pwmPin, LOW);
}

static void attach(const Motor &m) {
    pinMode(m.in1Pin, OUTPUT);
    pinMode(m.in2Pin, OUTPUT);
    pinMode(m.pwmPin, OUTPUT);
    release(m);
}

static void report(const char *phase) {
    for (int i = 0; i < SAMPLES_PER_PHASE; i++) {
        delay(SAMPLE_MS);
        Serial.printlnf("%-14s VM %.2f V   VCC %.2f V", phase,
                        readVolts(PIN_VM_SENSE) * VM_DIVIDER,
                        readVolts(PIN_VCC_SENSE));
    }
}

void setup() {
    attach(MOTOR_A);
    attach(MOTOR_B);

    pinMode(PIN_STBY, OUTPUT);
    digitalWrite(PIN_STBY, LOW);

    Serial.begin(115200);
    delay(1000);
    Serial.println("motor probe");
}

void loop() {
    digitalWrite(PIN_STBY, LOW);
    report("driver asleep");

    digitalWrite(PIN_STBY, HIGH);

    fullForward(MOTOR_A);
    report("motor A full");
    release(MOTOR_A);

    fullForward(MOTOR_B);
    report("motor B full");
    release(MOTOR_B);
}
