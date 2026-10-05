#ifndef MONARCH_STRATEGY_H
#define MONARCH_STRATEGY_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// Monarch of the Hill strategy: pure decision logic, no hardware access.
//
// Feed it the attitude estimate (see Attitude.h) every control tick and it
// returns left/right drive commands. Keeping it hardware-free is what lets the
// unit tests run it against a simulated ramp.
//
// The idea in one paragraph: whenever the chassis is on a slope, steer toward
// "uphill" as measured by gravity - which works no matter which way the robot
// is facing, so being shoved sideways or spun around just makes it turn back
// toward the summit. If uphill turns out to be BEHIND it (it rolled over the
// top and is now pointing down the far ramp) it simply backs up instead of
// turning around. Once the chassis reads level it has reached the mesa; the
// tilt sensor cannot say where on the 1 ft mesa that is, so it dead-reckons
// half the mesa length (in the direction it was travelling), then holds. If
// at any point it senses a slope again it goes back to climbing. If a nudge
// ever carries it over the far edge, the next nudge is half as long, so a bad
// speed estimate settles down instead of swinging between the two edges.
//
//   Waiting   motors off until it is sitting on a slope
//   Slope     steer to the fall line, climb (or back up if facing downhill)
//   Centering timed drive from the first flat reading to mid-mesa
//   Holding   stopped; re-aims if it gets turned, resumes Slope if pushed off
// ---------------------------------------------------------------------------

struct StrategyConfig {
    // --- slope detection (degrees off level) ---
    float slopeEnterDeg = 20.0f;     // ramp is 33 +/- 1; at or above this = on a slope
    float slopeExitDeg = 12.0f;      // below this = level (mesa). Gap = hysteresis.
    float bearingValidDeg = 8.0f;    // below this the uphill direction is too noisy to use
    uint32_t slopeConfirmMs = 80;    // slope must persist this long to count
    uint32_t flatConfirmMs = 150;    // level must persist this long to count

    // --- speeds (-255..255 motor command units) ---
    int driveSpeed = 200;
    int crestSpeed = 120;            // used once tilt drops below slowBelowTiltDeg
    float slowBelowTiltDeg = 26.0f;  // ease off approaching the crest so we don't fly over
    int centerSpeed = 120;           // speed for the mesa nudge; measure speed at THIS value

    // --- steering ---
    float kp = 2.0f;                 // command units per degree of heading error
    float kd = 0.6f;                 // command units per deg/s of turn rate (gyro damping)
    int turnMax = 110;
    float pivotEnterDeg = 55.0f;     // error this big: stop driving, spin in place
    float pivotExitDeg = 30.0f;
    int pivotSpeed = 150;
    float reverseEnterDeg = 130.0f;  // uphill is this far off the nose: back up instead
    float reverseExitDeg = 110.0f;

    // --- mesa dead reckoning ---
    float mesaLengthIn = 12.0f;               // from the contest rules
    float measuredSpeedInPerS = 0.0f;         // MEASURE at centerSpeed. 0 = use the fallback
    float centerOffsetIn = 0.0f;              // tune: + drives further, - less
    uint32_t fallbackCenterMs = 300;

    // --- holding ---
    float holdDeadbandDeg = 20.0f;   // re-aim only if turned this far. 0 disables re-aiming.
    float holdSettleDeg = 8.0f;
};

enum class Phase : uint8_t { Waiting, Slope, Centering, Holding };

struct StrategyInputs {
    uint32_t nowMs;
    float tiltDeg;
    float bearingDeg;       // uphill relative to nose, CCW+
    float yawDeg;           // gyro heading, CCW+
    float yawRateDps;       // CCW+
};

struct DriveCommand {
    int left;               // -255..255, positive = forward
    int right;
};

class Strategy {
public:
    explicit Strategy(const StrategyConfig &cfg = StrategyConfig());

    void begin(uint32_t nowMs);
    DriveCommand update(const StrategyInputs &in);

    Phase phase() const { return _phase; }
    const char *phaseName() const;
    uint32_t centerDriveMs() const;
    bool reversing() const { return _reversing; }

private:
    DriveCommand steer(float errDeg, float yawRateDps, int base) const;
    DriveCommand pivot(float errDeg) const;
    void enter(Phase p, uint32_t nowMs);

    StrategyConfig _cfg;
    Phase _phase;
    uint32_t _phaseStartMs;

    uint32_t _slopeSinceMs;
    uint32_t _flatSinceMs;
    bool _slopeTiming;
    bool _flatTiming;

    float _headingTargetDeg;
    float _holdHeadingDeg;
    int _centerDir;
    bool _reversing;
    bool _pivoting;
    bool _reaiming;
    float _nudgeScale;      // halves every time a nudge makes us fall off the mesa
};

#endif // MONARCH_STRATEGY_H
