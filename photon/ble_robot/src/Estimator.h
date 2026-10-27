#ifndef MONARCH_OF_THE_HILL_ESTIMATOR_H
#define MONARCH_OF_THE_HILL_ESTIMATOR_H

#include <math.h>

// ---------------------------------------------------------------------------
// Tilt estimation: which way is up, in the sensor's own frame.
//
// An accelerometer alone can't answer that while the robot is speeding up or
// slowing down - it measures gravity and acceleration added together and has
// no way to tell them apart, so 0.3 g of forward acceleration reads exactly
// like a 17 degree tilt. A gyro alone can: it only responds to rotation. But
// its small errors accumulate, so on its own the angle drifts off over time.
//
// So the estimate follows the gyro from moment to moment, and is pulled
// slowly toward the accelerometer only while the accelerometer is believable:
// reading close to 1 g, not spinning, and the motors not in the middle of
// changing speed. Through a launch or a hard stop it coasts on the gyro alone,
// which over a few seconds drifts by well under a degree.
//
// The estimate is a unit vector rather than an angle, so nothing here depends
// on how the sensor is mounted. Mounting turns that vector into pitch and
// roll, from two directions captured once: up with the robot level, and
// forward, found by tilting the nose up.
//
// Plain C++ with no Particle dependencies, so the maths can be exercised off
// the robot.
// ---------------------------------------------------------------------------

struct Vec3 {
    float x;
    float y;
    float z;

    Vec3 operator+(const Vec3 &o) const { return Vec3{x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3 &o) const { return Vec3{x - o.x, y - o.y, z - o.z}; }
    Vec3 operator*(float k) const { return Vec3{x * k, y * k, z * k}; }

    float dot(const Vec3 &o) const { return x * o.x + y * o.y + z * o.z; }

    Vec3 cross(const Vec3 &o) const {
        return Vec3{y * o.z - z * o.y, z * o.x - x * o.z, x * o.y - y * o.x};
    }

    float magnitude() const { return sqrtf(dot(*this)); }

    // Leaves a near-zero vector alone rather than dividing by nothing.
    Vec3 normalized() const {
        float m = magnitude();
        return m < 1e-6f ? *this : *this * (1.0f / m);
    }
};

static constexpr float RAD_TO_DEG_F = 57.29578f;
static constexpr float DEG_TO_RAD_F = 0.01745329f;

class TiltEstimator {
public:
    // How far from 1 g the accelerometer may read and still be believed.
    static constexpr float TRUST_G_BAND = 0.08f;
    // Above this rotation rate the accelerometer is ignored.
    static constexpr float TRUST_MAX_RATE = 30.0f * DEG_TO_RAD_F;
    // How quickly the estimate is pulled toward a trusted accelerometer.
    static constexpr float CORRECTION_TAU_S = 1.5f;

    // Starts the estimate from the accelerometer outright. Only meaningful
    // while the robot is sitting still.
    void reset(const Vec3 &accelG) { _up = accelG.normalized(); }

    // gyro in rad/s with its resting offset already removed, accel in g.
    // motorsSteady is the caller's knowledge that no speed change is under way.
    void update(const Vec3 &gyro, const Vec3 &accelG, float dt, bool motorsSteady) {
        // A direction fixed in the world, seen from a body turning at w,
        // moves by -w x v.
        _up = (_up - gyro.cross(_up) * dt).normalized();

        float size = accelG.magnitude();
        _trusted = motorsSteady && fabsf(size - 1.0f) < TRUST_G_BAND &&
                   gyro.magnitude() < TRUST_MAX_RATE;

        if (_trusted) {
            float gain = dt / CORRECTION_TAU_S;
            _up = (_up + (accelG * (1.0f / size) - _up) * gain).normalized();
        }
    }

    Vec3 up() const { return _up; }

    // Whether the last update used the accelerometer.
    bool accelTrusted() const { return _trusted; }

private:
    Vec3 _up{0.0f, 0.0f, 1.0f};
    bool _trusted = false;
};

// The robot's own up and forward directions, expressed in the sensor's frame.
struct Mounting {
    Vec3 up{0.0f, 0.0f, 1.0f};
    Vec3 forward{1.0f, 0.0f, 0.0f};
    bool forwardKnown = false;

    // Call with the robot sitting level.
    void setLevel(const Vec3 &upNow) {
        up = upNow.normalized();
        // Keep forward square to the new up.
        forward = (forward - up * forward.dot(up)).normalized();
    }

    // Call with the nose tilted up. Up leans toward the front of the robot
    // when the nose rises, so the part of it that is square to the level
    // direction points forward. Returns false if the tilt is too small to
    // give a reliable direction.
    bool setNoseUp(const Vec3 &upNow) {
        Vec3 lean = upNow.normalized() - up * upNow.normalized().dot(up);
        if (lean.magnitude() < MIN_LEAN) return false;

        forward = lean.normalized();
        forwardKnown = true;
        return true;
    }

    Vec3 left() const { return up.cross(forward); }

    // Degrees, nose up positive.
    float pitch(const Vec3 &upNow) const {
        return atan2f(upNow.dot(forward), upNow.dot(up)) * RAD_TO_DEG_F;
    }

    // Degrees, left side up positive.
    float roll(const Vec3 &upNow) const {
        return atan2f(upNow.dot(left()), upNow.dot(up)) * RAD_TO_DEG_F;
    }

private:
    static constexpr float MIN_LEAN = 0.087f;      // sine of 5 degrees
};


#endif //MONARCH_OF_THE_HILL_ESTIMATOR_H
