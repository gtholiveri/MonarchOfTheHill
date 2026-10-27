#include "Particle.h"

// ---------------------------------------------------------------------------
// Quiet firmware - flash this between tests.
//
// Puts the motor driver to sleep and then does nothing, so the motors stay
// silent while wiring is being worked on.
//
//   particle flash --local              (from this folder, Photon on USB)
// ---------------------------------------------------------------------------

// Connects to the cloud in the background so new code can be flashed over
// Wi-Fi. setup() and loop() start straight away and don't wait for it.
SYSTEM_MODE(AUTOMATIC);

static const pin_t PIN_STBY = D10;

void setup() {
    pinMode(PIN_STBY, OUTPUT);
    digitalWrite(PIN_STBY, LOW);
}

void loop() {
}
