#include "Strategy.h"

#include <math.h>

static float wrap180(float a) {
    a = fmodf(a, 360.0f);
    if (a > 180.0f) a -= 360.0f;
    if (a <= -180.0f) a += 360.0f;
    return a;
}

static float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static int clampCmd(float v) {
    return (int) lroundf(clampf(v, -255.0f, 255.0f));
}

Strategy::Strategy(const StrategyConfig &cfg)
    : _cfg(cfg),
      _phase(Phase::Waiting),
      _phaseStartMs(0),
      _slopeSinceMs(0),
      _flatSinceMs(0),
      _slopeTiming(false),
      _flatTiming(false),
      _headingTargetDeg(0),
      _holdHeadingDeg(0),
      _centerDir(1),
      _reversing(false),
      _pivoting(false),
      _reaiming(false),
      _nudgeScale(1.0f) {}

void Strategy::begin(uint32_t nowMs) {
    enter(Phase::Waiting, nowMs);
    _reversing = false;
    _pivoting = false;
    _slopeTiming = false;
    _flatTiming = false;
    _nudgeScale = 1.0f;
}

void Strategy::enter(Phase p, uint32_t nowMs) {
    _phase = p;
    _phaseStartMs = nowMs;
    _reaiming = false;
}

const char *Strategy::phaseName() const {
    switch (_phase) {
        case Phase::Waiting:   return "waiting";
        case Phase::Slope:     return "slope";
        case Phase::Centering: return "centering";
        case Phase::Holding:   return "holding";
    }
    return "?";
}

// time = distance / speed. Distance is half the mesa plus any tuning offset.
uint32_t Strategy::centerDriveMs() const {
    if (_cfg.measuredSpeedInPerS <= 0.0f) {
        return (uint32_t) (_cfg.fallbackCenterMs * _nudgeScale);
    }

    float distanceIn = _cfg.mesaLengthIn / 2.0f + _cfg.centerOffsetIn;
    float seconds = distanceIn / _cfg.measuredSpeedInPerS;
    if (seconds < 0.0f) seconds = 0.0f;
    return (uint32_t) (seconds * 1000.0f * _nudgeScale);
}

// Differential steer toward errDeg (positive = target is counter-clockwise of
// where we point). Rotation depends only on right-minus-left, so the same
// formula steers correctly whether base is forward or reverse.
DriveCommand Strategy::steer(float errDeg, float yawRateDps, int base) const {
    float turn = clampf(_cfg.kp * errDeg - _cfg.kd * yawRateDps,
                        (float) -_cfg.turnMax, (float) _cfg.turnMax);
    return DriveCommand{clampCmd(base - turn), clampCmd(base + turn)};
}

DriveCommand Strategy::pivot(float errDeg) const {
    int t = errDeg > 0 ? _cfg.pivotSpeed : -_cfg.pivotSpeed;
    return DriveCommand{-t, t};
}

DriveCommand Strategy::update(const StrategyInputs &in) {
    // Debounced slope / level detection.
    bool slopeNow = in.tiltDeg >= _cfg.slopeEnterDeg;
    bool flatNow = in.tiltDeg < _cfg.slopeExitDeg;

    if (slopeNow) {
        if (!_slopeTiming) { _slopeTiming = true; _slopeSinceMs = in.nowMs; }
    } else {
        _slopeTiming = false;
    }
    if (flatNow) {
        if (!_flatTiming) { _flatTiming = true; _flatSinceMs = in.nowMs; }
    } else {
        _flatTiming = false;
    }
    bool slopeConfirmed = slopeNow && (in.nowMs - _slopeSinceMs >= _cfg.slopeConfirmMs);
    bool flatConfirmed = flatNow && (in.nowMs - _flatSinceMs >= _cfg.flatConfirmMs);

    // 1. Phase changes first, so the command below always comes from the
    //    phase we are actually in (no one-tick "stop" glitch between phases).
    if (_phase != Phase::Slope && slopeConfirmed) {
        // The fall line, remembered in gyro-heading terms so it is still known
        // after the chassis levels out and the uphill direction disappears.
        _headingTargetDeg = wrap180(in.yawDeg + in.bearingDeg);
        _reversing = false;
        _pivoting = false;
        if (_phase == Phase::Centering) _nudgeScale *= 0.5f;   // the nudge put us over an edge
        enter(Phase::Slope, in.nowMs);
    } else if (_phase == Phase::Centering && in.nowMs - _phaseStartMs >= centerDriveMs()) {
        enter(Phase::Holding, in.nowMs);
    }

    switch (_phase) {
        case Phase::Waiting:
        case Phase::Centering:
        case Phase::Holding:
            break;

        case Phase::Slope:
            if (flatConfirmed) {
                _centerDir = _reversing ? -1 : 1;
                _holdHeadingDeg = _centerDir > 0 ? _headingTargetDeg
                                                 : wrap180(_headingTargetDeg + 180.0f);
                enter(Phase::Centering, in.nowMs);
            }
            break;
    }

    // 2. The command for the current phase.
    switch (_phase) {

        case Phase::Waiting:
            return DriveCommand{0, 0};

        case Phase::Slope: {
            if (in.tiltDeg >= _cfg.bearingValidDeg) {
                _headingTargetDeg = wrap180(in.yawDeg + in.bearingDeg);
            }

            float err = wrap180(_headingTargetDeg - in.yawDeg);
            float absErr = fabsf(err);
            if (!_reversing && absErr > _cfg.reverseEnterDeg) _reversing = true;
            else if (_reversing && absErr < _cfg.reverseExitDeg) _reversing = false;

            // Error measured from the end of the robot that is leading.
            float face = _reversing ? wrap180(err - 180.0f) : err;
            float absFace = fabsf(face);
            if (!_pivoting && absFace > _cfg.pivotEnterDeg) _pivoting = true;
            else if (_pivoting && absFace < _cfg.pivotExitDeg) _pivoting = false;

            if (_pivoting) return pivot(face);

            int speed = in.tiltDeg < _cfg.slowBelowTiltDeg ? _cfg.crestSpeed : _cfg.driveSpeed;
            return steer(face, in.yawRateDps, _reversing ? -speed : speed);
        }

        case Phase::Centering: {
            float err = wrap180(_holdHeadingDeg - in.yawDeg);
            return steer(err, in.yawRateDps, _centerDir * _cfg.centerSpeed);
        }

        case Phase::Holding: {
            if (_cfg.holdDeadbandDeg <= 0.0f) return DriveCommand{0, 0};

            float err = wrap180(_holdHeadingDeg - in.yawDeg);
            float absErr = fabsf(err);
            if (!_reaiming && absErr > _cfg.holdDeadbandDeg) _reaiming = true;
            else if (_reaiming && absErr < _cfg.holdSettleDeg) _reaiming = false;

            return _reaiming ? pivot(err) : DriveCommand{0, 0};
        }
    }
    return DriveCommand{0, 0};
}
