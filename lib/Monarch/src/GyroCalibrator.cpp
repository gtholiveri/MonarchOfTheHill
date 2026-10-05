#include "GyroCalibrator.h"

void GyroCalibrator::reset() {
    _n = 0;
    for (int i = 0; i < 3; i++) {
        _mean[i] = 0.0f;
        _m2[i] = 0.0f;
    }
}

// Welford's running mean/variance.
void GyroCalibrator::add(const Vec3 &g) {
    _n++;
    for (int i = 0; i < 3; i++) {
        float x = g.at(i);
        float delta = x - _mean[i];
        _mean[i] += delta / (float) _n;
        _m2[i] += delta * (x - _mean[i]);
    }
}

float GyroCalibrator::spreadDps() const {
    if (_n < 2) return 0.0f;
    float worst = 0.0f;
    for (int i = 0; i < 3; i++) {
        float sd = sqrtf(_m2[i] / (float) (_n - 1));
        if (sd > worst) worst = sd;
    }
    return worst;
}

bool GyroCalibrator::still(int minSamples, float maxSpreadDps) const {
    return _n >= minSamples && spreadDps() <= maxSpreadDps;
}
