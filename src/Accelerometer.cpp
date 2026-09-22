//
// Created by gabeo on 9/21/2026.
//

#include <Wire.h>

#include "Accelerometer.h"

Accelerometer::Accelerometer(range_t range)
    : _adxl(12345), _range(range) {}    // the int is just a sensor ID for logging

// Weak pull-ups make SDA rise slowly, which breaks multi-byte reads long
// before it breaks the short address handshake - the failure looks like
// "ACKs in a bus scan, times out on every register read". 50 kHz gives the
// line four times as long to settle.
static const uint32_t I2C_CLOCK_HZ = 50000;
static const int BEGIN_ATTEMPTS = 5;

bool Accelerometer::begin(int sdaPin, int sclPin) {
    if (sdaPin >= 0 && sclPin >= 0) {
        Wire.begin(sdaPin, sclPin);
    } else {
        Wire.begin();
    }

    Wire.setClock(I2C_CLOCK_HZ);
    Wire.setTimeOut(100);

    for (int attempt = 0; attempt < BEGIN_ATTEMPTS; attempt++) {
        if (_adxl.begin()) {            // checks DEVID == 0xE5
            Wire.setClock(I2C_CLOCK_HZ);  // driver re-inits Wire, so reassert
            _adxl.setRange(_range);       // also sets FULL_RES, 4 mg/LSB holds
            return true;
        }
        delay(50);
    }
    return false;
}

Vec3 Accelerometer::read() {
    sensors_event_t event;
    _adxl.getEvent(&event);

    return Vec3{event.acceleration.x,
                event.acceleration.y,
                event.acceleration.z};
}

Vec3 Accelerometer::readG() {
    Vec3 a = read();
    const float g = SENSORS_GRAVITY_STANDARD;
    return Vec3{a.x / g, a.y / g, a.z / g};
}

Vec3 Accelerometer::readRaw() {
    return Vec3{(float) _adxl.getX(), (float) _adxl.getY(), (float) _adxl.getZ()};
}
