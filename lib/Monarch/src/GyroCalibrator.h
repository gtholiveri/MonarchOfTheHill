#ifndef MONARCH_GYRO_CALIBRATOR_H
#define MONARCH_GYRO_CALIBRATOR_H

#include "Vec3.h"

// ---------------------------------------------------------------------------
// Measures the gyro's zero-rate offset while the robot sits still, so it can
// be subtracted later. A gyro that reads 1 deg/s while motionless would
// otherwise make the heading drift 30 degrees over a 30 second match.
//
// Feed it samples with add(). still() says whether the readings were calm
// enough to trust - if the robot was being handled while it averaged, the
// offset would be garbage, and the caller should keep sampling or give up.
// ---------------------------------------------------------------------------
class GyroCalibrator {
public:
    GyroCalibrator() { reset(); }

    void reset();
    void add(const Vec3 &gyroDps);

    int count() const { return _n; }
    Vec3 bias() const { return Vec3{_mean[0], _mean[1], _mean[2]}; }

    // Largest per-axis standard deviation seen so far, in deg/s.
    float spreadDps() const;

    // True once enough samples are in AND they were steady.
    bool still(int minSamples, float maxSpreadDps) const;

private:
    int _n;
    float _mean[3];
    float _m2[3];
};

#endif // MONARCH_GYRO_CALIBRATOR_H
