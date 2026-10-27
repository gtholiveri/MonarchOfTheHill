#include "Particle.h"

// ---------------------------------------------------------------------------
// Photon 2 motor hold test - temporary, for measuring with a multimeter.
//
// Sets motor A to full speed forward and leaves it there. Every pin is a
// steady level and nothing uses PWM, so there is no buzz and each point can
// be probed at leisure. Driver is the SparkFun TB6612FNG.
//
//   particle flash --local              (from this folder, Photon on USB)
//
// Expected, COM probe on GND:
//
//   VCC   3.3 V      STBY  3.3 V
//   VM    supply     AIN1  3.3 V
//   AO1   ~ VM       AIN2  0 V
//   AO2   ~ 0 V      PWMA  3.3 V
// ---------------------------------------------------------------------------

// Connects to the cloud in the background so new code can be flashed over
// Wi-Fi. setup() and loop() start straight away and don't wait for it.
SYSTEM_MODE(AUTOMATIC);

static const pin_t PIN_AIN1 = D5;
static const pin_t PIN_AIN2 = D6;
static const pin_t PIN_PWMA = MOSI;
static const pin_t PIN_STBY = D10;

void setup() {
    pinMode(PIN_AIN1, OUTPUT);
    pinMode(PIN_AIN2, OUTPUT);
    pinMode(PIN_PWMA, OUTPUT);
    pinMode(PIN_STBY, OUTPUT);

    digitalWrite(PIN_AIN1, HIGH);
    digitalWrite(PIN_AIN2, LOW);
    digitalWrite(PIN_PWMA, HIGH);
    digitalWrite(PIN_STBY, HIGH);

    Serial.begin(115200);
}

void loop() {
    Serial.println("motor A held at full forward: AIN1 high, AIN2 low, PWMA high, STBY high");
    delay(2000);
}
