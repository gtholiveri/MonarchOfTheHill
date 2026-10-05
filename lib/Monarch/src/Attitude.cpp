#include "Attitude.h"

static const float DEG = 57.29577951308232f;   // degrees per radian

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static Vec3 normalized(const Vec3 &v) {
    float n = v.norm();
    if (n < 1e-6f) return Vec3{0, 0, 1};
    return v * (1.0f / n);
}

Attitude::Attitude(const AttitudeConfig &cfg)
    : _cfg(cfg), _up{0, 0, 1}, _yawDeg(0), _yawRateDps(0), _pitchRateDps(0) {}

void Attitude::reset(const Vec3 &accelG) {
    _up = normalized(accelG);
    _yawDeg = 0;
    _yawRateDps = 0;
    _pitchRateDps = 0;
}

void Attitude::update(const Vec3 &accelG, const Vec3 &gyroDps, float dt) {
    if (dt <= 0.0f) return;

    // 1. Gyro step. A direction fixed in the world (gravity) appears to
    //    rotate the opposite way when seen from the spinning body:
    //    d(up)/dt = -omega x up.
    Vec3 omega = gyroDps * (1.0f / DEG);                   // rad/s
    _up = normalized(_up - cross(omega, _up) * dt);

    // 2. Accelerometer pull toward measured gravity, faded out whenever the
    //    reading's magnitude says it is not just gravity.
    float mag = accelG.norm();
    if (mag > 0.1f) {
        float gate = 1.0f - fabsf(mag - 1.0f) / _cfg.accelGateG;
        if (gate > 0.0f) {
            Vec3 measured = accelG * (1.0f / mag);
            float k = clampf(_cfg.accelGain * gate * dt, 0.0f, 1.0f);
            _up = normalized(_up + (measured - _up) * k);
        }
    }

    // 3. Rates. Heading changes by the part of the spin that is about the
    //    world-vertical axis, which on a slope is not the body Z axis.
    _yawRateDps = dot(gyroDps, _up);
    _yawDeg += _yawRateDps * dt;
    // Nose-up rotation is negative about +Y (right-hand rule, Y to the left).
    _pitchRateDps = -gyroDps.y;
}

float Attitude::tiltDeg() const {
    return acosf(clampf(_up.z, -1.0f, 1.0f)) * DEG;
}

float Attitude::pitchDeg() const {
    return asinf(clampf(_up.x, -1.0f, 1.0f)) * DEG;
}

float Attitude::rollDeg() const {
    return asinf(clampf(_up.y, -1.0f, 1.0f)) * DEG;
}

float Attitude::uphillBearingDeg() const {
    return atan2f(_up.y, _up.x) * DEG;
}
