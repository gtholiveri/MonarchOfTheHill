//
// Created by gabeo on 9/21/2026.
//

#ifndef MONARCH_OF_THE_HILL_ACCELEROMETER_H
#define MONARCH_OF_THE_HILL_ACCELEROMETER_H

#include <Arduino.h>
#include <Adafruit_ADXL345_U.h>

// ---------------------------------------------------------------------------
// ADXL345 triple-axis accelerometer over I2C, wrapped to hand back a vector.
//
//   Accelerometer accel;            // wired to SDA/SCL
//   accel.begin();
//   Vec3 g = accel.read();          // g, one earth gravity = 1.0
//
// Built on Adafruit's driver rather than SparkFun's: SparkFun's readFromI2C()
// wraps a Wire.requestFrom() inside an open beginTransmission(), which is
// AVR-era usage that hangs ESP32's I2C driver on the first register read.
//
// The ADXL345 measures proper acceleration, so at rest one axis reads ~1g
// (whichever points up) rather than zero. read() is a blocking register read
// over the bus - fine at loop speed, don't call it in an ISR.
// ---------------------------------------------------------------------------

struct Vec3 {
    float x;
    float y;
    float z;

    float magnitude() const { return sqrtf(x * x + y * y + z * z); }
};

class Accelerometer {
public:
    explicit Accelerometer(range_t range = ADXL345_RANGE_16_G);

    // Returns false if the sensor doesn't answer - the driver verifies the
    // DEVID register, so a true here means the chip really is an ADXL345.
    // Pass explicit pins if the board doesn't use the default SDA/SCL.
    bool begin(int sdaPin = -1, int sclPin = -1);

    Vec3 read();        // acceleration in m/s^2 - at rest |a| ~= 9.81
    Vec3 readG();       // same, in g - at rest |a| ~= 1.00
    Vec3 readRaw();     // raw counts, unscaled

private:
    Adafruit_ADXL345_Unified _adxl;
    range_t _range;
};


#endif //MONARCH_OF_THE_HILL_ACCELEROMETER_H
