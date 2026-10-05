#include "Mpu6050.h"

#include <Wire.h>

// Register map (MPU-6000/6050 Register Map and Descriptions).
static const uint8_t REG_SMPLRT_DIV = 0x19;
static const uint8_t REG_CONFIG = 0x1A;
static const uint8_t REG_GYRO_CONFIG = 0x1B;
static const uint8_t REG_ACCEL_CONFIG = 0x1C;
static const uint8_t REG_ACCEL_XOUT_H = 0x3B;    // 14 bytes: accel XYZ, temp, gyro XYZ
static const uint8_t REG_PWR_MGMT_1 = 0x6B;
static const uint8_t REG_WHO_AM_I = 0x75;

// Ranges chosen here, and the matching conversion factors.
//   gyro  +/-500 deg/s  -> 65.5 LSB per deg/s
//   accel +/-4 g        -> 8192 LSB per g
// +/-500 deg/s is plenty for a 1 kg tracked robot; +/-4 g covers a hard hit.
static const uint8_t GYRO_FS_500 = 0x08;
static const uint8_t ACCEL_FS_4G = 0x08;
static const float GYRO_LSB_PER_DPS = 65.5f;
static const float ACCEL_LSB_PER_G = 8192.0f;

// Low-pass filter inside the chip: 44 Hz accelerometer / 42 Hz gyro bandwidth.
// Cuts motor vibration without adding much delay for a robot this slow.
static const uint8_t DLPF_44HZ = 0x03;

bool Mpu6050::writeReg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

bool Mpu6050::readRegs(uint8_t reg, uint8_t *buf, size_t n) {
    Wire.beginTransmission(_addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return false;     // repeated start

    size_t got = Wire.requestFrom((int) _addr, (int) n);
    if (got != n) return false;

    for (size_t i = 0; i < n; i++) buf[i] = Wire.read();
    return true;
}

bool Mpu6050::begin(uint8_t addr, int sdaPin, int sclPin, uint32_t clockHz) {
    _addr = addr;

    Wire.begin(sdaPin, sclPin);
    Wire.setClock(clockHz);
    Wire.setTimeOut(50);

    if (!readRegs(REG_WHO_AM_I, &_whoAmI, 1)) return false;

    // A genuine MPU-6050 answers 0x68. Cheap "GY-521" boards often carry a
    // pin-compatible clone that answers 0x70, 0x72 or 0x98 and behaves the
    // same for everything used here, so those are accepted too.
    bool known = _whoAmI == 0x68 || _whoAmI == 0x70 || _whoAmI == 0x72 || _whoAmI == 0x98;
    if (!known) return false;

    // Reset, then wake with the gyro X PLL as clock (steadier than the
    // internal oscillator).
    if (!writeReg(REG_PWR_MGMT_1, 0x80)) return false;
    delay(100);
    if (!writeReg(REG_PWR_MGMT_1, 0x01)) return false;
    delay(10);

    return writeReg(REG_SMPLRT_DIV, 4)          // 1 kHz / (1+4) = 200 Hz internal rate
           && writeReg(REG_CONFIG, DLPF_44HZ)
           && writeReg(REG_GYRO_CONFIG, GYRO_FS_500)
           && writeReg(REG_ACCEL_CONFIG, ACCEL_FS_4G);
}

bool Mpu6050::read(ImuSample &out) {
    uint8_t b[14];
    if (!readRegs(REG_ACCEL_XOUT_H, b, sizeof(b))) return false;

    auto s16 = [&](int i) -> int16_t { return (int16_t) ((b[i] << 8) | b[i + 1]); };

    out.accelG = Vec3{s16(0) / ACCEL_LSB_PER_G,
                      s16(2) / ACCEL_LSB_PER_G,
                      s16(4) / ACCEL_LSB_PER_G};
    // bytes 6-7 are the die temperature, not used
    out.gyroDps = Vec3{s16(8) / GYRO_LSB_PER_DPS,
                       s16(10) / GYRO_LSB_PER_DPS,
                       s16(12) / GYRO_LSB_PER_DPS};
    return true;
}
