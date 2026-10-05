#ifndef MONARCH_MPU6050_H
#define MONARCH_MPU6050_H

#include <Arduino.h>

#include "Vec3.h"

// ---------------------------------------------------------------------------
// MPU-6050: 3-axis gyro + 3-axis accelerometer on one chip, over I2C.
//
// Talks to the chip's registers directly through Wire instead of pulling in a
// library. The part is simple (wake it, pick ranges, read 14 bytes) and a
// direct driver means no library-version surprises and one burst read gives
// the accelerometer and gyro from the same instant.
//
// Output is in the SENSOR's own axes. Remapping to the robot's frame is done
// by the caller (see config.h).
// ---------------------------------------------------------------------------

struct ImuSample {
    Vec3 accelG;      // gravity units, 1.0 = 1 g
    Vec3 gyroDps;     // degrees per second
};

class Mpu6050 {
public:
    // Starts I2C on the given pins and configures the chip. Returns false if
    // the chip doesn't answer or doesn't identify itself as an MPU-6050 family
    // part.
    bool begin(uint8_t addr, int sdaPin, int sclPin, uint32_t clockHz);

    // One burst read. Returns false on a bus error.
    bool read(ImuSample &out);

    uint8_t whoAmI() const { return _whoAmI; }

private:
    bool writeReg(uint8_t reg, uint8_t value);
    bool readRegs(uint8_t reg, uint8_t *buf, size_t n);

    uint8_t _addr = 0x68;
    uint8_t _whoAmI = 0;
};

#endif // MONARCH_MPU6050_H
