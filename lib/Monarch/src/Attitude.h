#ifndef MONARCH_ATTITUDE_H
#define MONARCH_ATTITUDE_H

#include "Vec3.h"

// ---------------------------------------------------------------------------
// Gyro + accelerometer fusion (a complementary filter on the gravity vector).
//
// Why both sensors: the accelerometer alone gives an absolute "which way is
// down", but it also feels every push, bump and motor surge, so an
// accelerometer-only tilt reading jumps around exactly when the robot is
// accelerating up the ramp or getting shoved. The gyro is smooth and fast but
// only measures rotation *rate*, so integrating it alone drifts. This filter
// trusts the gyro moment to moment and lets the accelerometer slowly pull the
// estimate back toward true gravity, ignoring it while the reading doesn't
// look like 1 g (i.e. while something other than gravity is accelerating us).
//
// Everything here is in the BODY frame: +X forward, +Y left, +Z up (when the
// chassis is level). Map the raw sensor axes into that frame before calling
// update() - see config.h.
//
// What it hands back is what the strategy needs:
//   tiltDeg()          how far the chassis is off level, any direction (0..180)
//   uphillBearingDeg() which way is uphill relative to the nose: 0 = dead
//                      ahead, +90 = to the left, 180 = behind. Only meaningful
//                      while tilted (on flat ground there is no uphill).
//   yawDeg()           heading from integrating the gyro, CCW positive,
//                      relative to where it was at reset(). Drifts slowly.
//   yawRateDps()       turn rate about the vertical axis, CCW positive.
// ---------------------------------------------------------------------------

struct AttitudeConfig {
    // How hard the accelerometer pulls the estimate toward measured gravity,
    // in 1/seconds. ~1.5 means the estimate corrects over roughly 0.7 s.
    float accelGain = 1.5f;

    // The accelerometer is only trusted while | |a| - 1 g | is under this.
    // Trust fades linearly to zero at the limit.
    float accelGateG = 0.25f;
};

class Attitude {
public:
    explicit Attitude(const AttitudeConfig &cfg = AttitudeConfig());

    // Seed from one stationary accelerometer reading (in g, body frame).
    void reset(const Vec3 &accelG);

    // accelG: g, body frame. gyroDps: deg/s, body frame, bias already removed.
    // dt: seconds since the previous call.
    void update(const Vec3 &accelG, const Vec3 &gyroDps, float dt);

    float tiltDeg() const;
    float pitchDeg() const;          // nose-up positive
    float rollDeg() const;           // left-side-up positive
    float uphillBearingDeg() const;
    float yawDeg() const { return _yawDeg; }
    float yawRateDps() const { return _yawRateDps; }
    float pitchRateDps() const { return _pitchRateDps; }   // nose-up positive

    // World "up" expressed in the body frame, unit length.
    const Vec3 &up() const { return _up; }

private:
    AttitudeConfig _cfg;
    Vec3 _up;
    float _yawDeg;
    float _yawRateDps;
    float _pitchRateDps;
};

#endif // MONARCH_ATTITUDE_H
