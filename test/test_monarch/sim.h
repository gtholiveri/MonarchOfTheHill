#ifndef MONARCH_SIM_H
#define MONARCH_SIM_H

// A kinematic stand-in for the robot on the contest hill, used only by the
// unit tests. The true orientation is built from rotation matrices and the
// fake gyro / accelerometer readings are derived from it independently of
// the filter code under test, so a sign error in Attitude cannot hide by
// agreeing with itself.

#include <math.h>
#include <stdint.h>

#include "Attitude.h"
#include "Strategy.h"

static const float RAD = 0.017453292519943295f;
static const float DEGS = 57.29577951308232f;

struct M3 { float m[3][3]; };

inline M3 mul(const M3 &a, const M3 &b) {
    M3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) {
            r.m[i][j] = 0;
            for (int k = 0; k < 3; k++) r.m[i][j] += a.m[i][k] * b.m[k][j];
        }
    return r;
}
inline M3 transpose(const M3 &a) {
    M3 r;
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++) r.m[i][j] = a.m[j][i];
    return r;
}
inline Vec3 mulv(const M3 &a, const Vec3 &v) {
    return Vec3{a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
                a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
                a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z};
}
inline M3 Ry(float a) {
    float c = cosf(a), s = sinf(a);
    return M3{{{c, 0, s}, {0, 1, 0}, {-s, 0, c}}};
}
inline M3 Rz(float a) {
    float c = cosf(a), s = sinf(a);
    return M3{{{c, -s, 0}, {s, c, 0}, {0, 0, 1}}};
}

// Angular velocity (body frame, rad/s) that takes Rprev to Rcur over dt.
inline Vec3 bodyRate(const M3 &Rprev, const M3 &Rcur, float dt) {
    M3 d = mul(transpose(Rprev), Rcur);
    return Vec3{0.5f * (d.m[2][1] - d.m[1][2]) / dt,
                0.5f * (d.m[0][2] - d.m[2][0]) / dt,
                0.5f * (d.m[1][0] - d.m[0][1]) / dt};
}

// Deterministic roughly-gaussian noise.
struct Noise {
    uint32_t s = 12345;
    float uni() {
        s = s * 1664525u + 1013904223u;
        return (float) (s >> 8) / 16777216.0f;
    }
    float gauss() {
        float a = 0;
        for (int i = 0; i < 12; i++) a += uni();
        return a - 6.0f;
    }
};

// The hill, measured along its surface in inches: flat approach, 60 in ramp
// at 33 deg, 12 in mesa, 60 in ramp down, flat after.
struct Hill {
    float slopeDeg = 33.0f;
    float rampIn = 60.0f;
    float mesaIn = 12.0f;

    float height(float l) const {
        float s = sinf(slopeDeg * RAD);
        if (l <= 0) return 0;
        if (l <= rampIn) return l * s;
        float top = rampIn * s;
        if (l <= rampIn + mesaIn) return top;
        float d = l - rampIn - mesaIn;
        if (d <= rampIn) return top - d * s;
        return 0;
    }
    float mesaStart() const { return rampIn; }
    float mesaEnd() const { return rampIn + mesaIn; }
    float mesaCenter() const { return rampIn + mesaIn / 2.0f; }
};

struct Sim {
    Hill hill;
    float chassisLenIn = 8.0f;
    float trackIn = 6.0f;
    float vmaxInPerS = 16.0f;        // at command 255
    float motorLagS = 0.08f;

    float ell = 6.0f;                // CG position along the surface
    float psi = 0.0f;                // heading relative to fall line, rad, CCW+
    float vL = 0, vR = 0, vPrev = 0;

    float dt = 0.01f;
    uint32_t nowMs = 0;
    float gyroBiasDps = 0.0f;        // leftover bias AFTER the boot-time calibration
    float accelNoiseG = 0.01f;
    float gyroNoiseDps = 0.3f;
    float vibrationG = 0.04f;        // extra accel noise while driving

    Noise noise;
    Attitude att;
    Strategy strat;
    DriveCommand cmd{0, 0};
    M3 Rprev;
    Vec3 lastAccel{0, 0, 1};
    float maxTiltSeen = 0;

    explicit Sim(const StrategyConfig &sc = StrategyConfig(),
                 const AttitudeConfig &ac = AttitudeConfig())
        : att(ac), strat(sc) {}

    float pitchAt(float l) const {
        // The two contact points are chassisLenIn apart ALONG the surface, so
        // the height difference is the sine of the tilt, not the tangent.
        float h = hill.height(l + chassisLenIn / 2) - hill.height(l - chassisLenIn / 2);
        float r = h / chassisLenIn;
        if (r > 1.0f) r = 1.0f;
        if (r < -1.0f) r = -1.0f;
        return asinf(r);
    }
    M3 orientation() const { return mul(Ry(-pitchAt(ell)), Rz(psi)); }

    void start() {
        Rprev = orientation();
        Vec3 up = mulv(transpose(Rprev), Vec3{0, 0, 1});
        att.reset(up);
        strat.begin(0);
    }

    void step() {
        float tl = cmd.left / 255.0f * vmaxInPerS;
        float tr = cmd.right / 255.0f * vmaxInPerS;
        float a = dt / motorLagS;
        vL += (tl - vL) * a;
        vR += (tr - vR) * a;
        float v = 0.5f * (vL + vR);
        float w = (vR - vL) / trackIn;

        psi += w * dt;
        ell += v * cosf(psi) * dt;
        if (ell < -20) ell = -20;
        if (ell > 150) ell = 150;

        M3 R = orientation();
        Vec3 up = mulv(transpose(R), Vec3{0, 0, 1});
        float surge = (v - vPrev) / dt / 386.09f;        // g, along the nose
        vPrev = v;

        float vib = (fabsf(v) > 0.5f ? vibrationG : 0.0f);
        Vec3 accel{up.x + surge + noise.gauss() * (accelNoiseG + vib),
                   up.y + noise.gauss() * (accelNoiseG + vib),
                   up.z + noise.gauss() * (accelNoiseG + vib)};
        Vec3 g = bodyRate(Rprev, R, dt) * DEGS;
        Vec3 gyro{g.x + gyroBiasDps + noise.gauss() * gyroNoiseDps,
                  g.y + gyroBiasDps + noise.gauss() * gyroNoiseDps,
                  g.z + gyroBiasDps + noise.gauss() * gyroNoiseDps};
        Rprev = R;
        lastAccel = accel;

        att.update(accel, gyro, dt);
        if (att.tiltDeg() > maxTiltSeen) maxTiltSeen = att.tiltDeg();

        nowMs += (uint32_t) lroundf(dt * 1000.0f);
        if (nowMs % 40 == 0) {
            StrategyInputs in;
            in.nowMs = nowMs;
            in.tiltDeg = att.tiltDeg();
            in.bearingDeg = att.uphillBearingDeg();
            in.yawDeg = att.yawDeg();
            in.yawRateDps = att.yawRateDps();
            cmd = strat.update(in);
        }
    }

    void run(float seconds) {
        int n = (int) (seconds / dt);
        for (int i = 0; i < n; i++) step();
    }

    // Run until the strategy reaches `p` or time runs out. Returns success.
    bool runUntil(Phase p, float maxSeconds) {
        int n = (int) (maxSeconds / dt);
        for (int i = 0; i < n; i++) {
            step();
            if (strat.phase() == p) return true;
        }
        return false;
    }
};

#endif // MONARCH_SIM_H
