#ifndef MONARCH_OF_THE_HILL_AUTOPILOT_H
#define MONARCH_OF_THE_HILL_AUTOPILOT_H

#include <stdint.h>

// ---------------------------------------------------------------------------
// The autonomous run: climb the ramp, cut the power, hold the top.
//
// The hill is two 5 ft ramps at 33 degrees meeting at a 1 ft flat top, and the
// robot starts at the bottom of one, already nose-up. It has no way to measure
// distance, so the climb is paced by time, tuned from test runs:
//
//   Climb   full power from the start
//   Brake   power falls away in a straight line to nothing, beginning at a
//           set time into the run and lasting a set time; a length of zero is
//           a hard stop
//   Hold    braked while level; push forward hard if the nose rises (short of
//           the top, or being shoved back down our own ramp), back up gently
//           if it drops (about to go over the far side)
//
// Pitch is the backstop on the timing: if the nose comes down over the ridge
// before the set time, braking starts then instead.
//
// Braking early is self-correcting. If the robot stops short it is still
// nose-up, so Hold drives it forward until it is level.
//
// Every number is a parameter, set from the dashboard and kept in EEPROM.
// Outputs are in robot terms (left and right, forward positive); the caller
// applies trim and maps them onto the actual motors.
//
// Plain C++ with no Particle dependencies.
// ---------------------------------------------------------------------------

enum ParamId {
    P_CLIMB_POWER,          // 0..255
    P_BRAKE_START_MS,       // into the run
    P_BRAKE_RAMP_MS,        // how long the power takes to fall to zero; 0 is a hard stop
    P_CREST_PITCH,          // degrees; below this the nose is over the ridge
    P_HOLD_BAND,            // degrees either side of level that count as "on top"
    P_HOLD_FORWARD_POWER,   // 0..255, used when the nose is up
    P_HOLD_BACK_POWER,      // 0..255, used when the nose is down
    P_MAX_RUN_S,            // whole run ends after this; 0 means never
    P_TRIM_LEFT,            // tenths of a percent, 0..1000
    P_TRIM_RIGHT,
    P_YAW_GAIN,             // tenths of a power count per degree off heading
    P_FLAGS,                // FLAG_* bits
    P_START_MODE,           // START_* : what begins a run
    P_LAUNCH_MG,            // thousandths of a g that count as the launch, in START_MOTION
    PARAM_COUNT
};

static const int FLAG_SWAP_SIDES = 0x01;
static const int FLAG_INVERT_LEFT = 0x02;
static const int FLAG_INVERT_RIGHT = 0x04;
static const int FLAG_STOP_ON_LINK_LOSS = 0x08;
static const int FLAG_ARM_ON_BOOT = 0x10;

// What begins a run. The rules want one press of a control on the robot and
// then hands off, so there are two ways to do that besides the dashboard.
//
//   START_DASHBOARD  the dashboard's Start button.
//   START_MOTION     the switch stays where it was first wired, on the
//                    driver's standby pin. Once armed the robot asks for full
//                    power the whole time; the driver is asleep, so nothing
//                    moves until the switch wakes it. The robot can't see the
//                    switch, so it takes the jolt of setting off, felt by the
//                    accelerometer, as the start of the run.
//   START_SWITCH     the switch is wired to a pin on the Photon instead, and
//                    standby to another. The robot reads the switch directly,
//                    holds itself on the ramp with the brakes while armed, and
//                    starts the instant the switch goes on.
static const int START_DASHBOARD = 0;
static const int START_MOTION = 1;
static const int START_SWITCH = 2;

struct ParamSpec {
    int16_t fallback;
    int16_t lowest;
    int16_t highest;
};

static const ParamSpec PARAM_SPECS[PARAM_COUNT] = {
    {255, 0, 255},              // P_CLIMB_POWER
    {3000, 0, 30000},           // P_BRAKE_START_MS
    {150, 0, 5000},             // P_BRAKE_RAMP_MS
    {18, -90, 60},              // P_CREST_PITCH
    {6, 1, 30},                 // P_HOLD_BAND
    {255, 0, 255},              // P_HOLD_FORWARD_POWER
    {110, 0, 255},              // P_HOLD_BACK_POWER
    {60, 0, 600},               // P_MAX_RUN_S
    {1000, 0, 1000},            // P_TRIM_LEFT
    {1000, 0, 1000},            // P_TRIM_RIGHT
    {0, 0, 300},                // P_YAW_GAIN
    {FLAG_STOP_ON_LINK_LOSS, 0, 31},    // P_FLAGS
    {START_DASHBOARD, 0, 2},    // P_START_MODE
    {80, 20, 1000},             // P_LAUNCH_MG
};

struct AutoParams {
    int16_t values[PARAM_COUNT];

    AutoParams() { reset(); }

    void reset() {
        for (int i = 0; i < PARAM_COUNT; i++) values[i] = PARAM_SPECS[i].fallback;
    }

    int operator[](ParamId id) const { return values[id]; }

    bool flag(int bit) const { return (values[P_FLAGS] & bit) != 0; }

    // Clamped to the parameter's range. Returns true if the value changed.
    bool set(int id, long value) {
        if (id < 0 || id >= PARAM_COUNT) return false;

        if (value < PARAM_SPECS[id].lowest) value = PARAM_SPECS[id].lowest;
        if (value > PARAM_SPECS[id].highest) value = PARAM_SPECS[id].highest;
        if (values[id] == (int16_t) value) return false;

        values[id] = (int16_t) value;
        return true;
    }
};

// Armed is never entered by the Autopilot itself. Waiting for the start
// depends on the switch and the accelerometer, which belong to the caller; it
// is listed here so the reported state has one numbering.
enum class AutoState : uint8_t { Idle, Climb, Brake, Hold, Armed };

enum class EndReason : uint8_t {
    None,
    StopCommand,
    ManualTakeover,
    LinkLost,
    RunTimeOver,
    SensorMissing,
    SwitchOff,
};

class Autopilot {
public:
    // Ignore the crest test this long after starting, so the estimate has
    // settled and a bench start on flat ground still shows a climb.
    static const unsigned long MIN_CLIMB_MS = 300;
    // The pitch must stay below the threshold this long to count as the crest.
    static const unsigned long CREST_CONFIRM_MS = 40;

    bool active() const { return _state != AutoState::Idle; }
    AutoState state() const { return _state; }
    EndReason lastEnd() const { return _lastEnd; }

    unsigned long msInState(unsigned long now) const { return active() ? now - _stateStart : 0; }
    unsigned long msInRun(unsigned long now) const { return active() ? now - _runStart : 0; }

    // untimed runs ignore P_MAX_RUN_S and go on until something stops them.
    void start(unsigned long now, bool untimed = false) {
        _runStart = now;
        _untimed = untimed;
        _belowCrestSince = 0;
        _push = 0;
        _lastEnd = EndReason::None;
        enter(AutoState::Climb, now);
    }

    void stop(EndReason why) {
        if (!active()) return;
        _state = AutoState::Idle;
        _lastEnd = why;
    }

    // pitchDeg is nose-up positive; yawDeg is how far the robot has turned
    // left since the run started. left and right come back as -255..255.
    void update(unsigned long now, float pitchDeg, float yawDeg, const AutoParams &p,
                int &left, int &right) {
        left = 0;
        right = 0;
        if (!active()) return;

        if (!_untimed && p[P_MAX_RUN_S] > 0 &&
            now - _runStart >= (unsigned long) p[P_MAX_RUN_S] * 1000UL) {
            stop(EndReason::RunTimeOver);
            return;
        }

        // The brake time is read fresh every pass, so moving it to "now" from
        // the dashboard mid-climb starts the braking at once.
        if (_state == AutoState::Climb &&
            (now - _runStart >= (unsigned long) p[P_BRAKE_START_MS] || crested(now, pitchDeg, p))) {
            enter(AutoState::Brake, now);
        }
        if (_state == AutoState::Brake && now - _stateStart >= (unsigned long) p[P_BRAKE_RAMP_MS]) {
            enter(AutoState::Hold, now);
        }

        int base = 0;
        switch (_state) {
            case AutoState::Climb:
                base = p[P_CLIMB_POWER];
                break;
            case AutoState::Brake: {
                // Straight line from full power down to nothing. A ramp of
                // zero never gets here: it moved on to Hold just above.
                float remaining = 1.0f - (float) (now - _stateStart) / (float) p[P_BRAKE_RAMP_MS];
                base = (int) (p[P_CLIMB_POWER] * remaining);
                break;
            }
            case AutoState::Hold:
                base = holdPower(pitchDeg, p);
                break;
            default:
                break;
        }

        // Heading hold on the way up: having turned left, the left side must
        // outrun the right to turn back. Left alone in Hold, where being
        // shoved around makes the starting heading meaningless.
        int correction = 0;
        if (_state != AutoState::Hold && base != 0) {
            correction = (int) (yawDeg * p[P_YAW_GAIN] / 10.0f);
        }

        left = base + correction;
        right = base - correction;

        // At full power there is no headroom to speed one side up, so the
        // whole correction comes off the other side instead. Clamping each
        // side alone would silently halve the steering.
        int over = (left > right ? left : right) - 255;
        if (over > 0) {
            left -= over;
            right -= over;
        }
        left = clampPower(left);
        right = clampPower(right);
    }

private:
    void enter(AutoState next, unsigned long now) {
        _state = next;
        _stateStart = now;
    }

    bool crested(unsigned long now, float pitchDeg, const AutoParams &p) {
        if (now - _runStart < MIN_CLIMB_MS || pitchDeg >= p[P_CREST_PITCH]) {
            _belowCrestSince = 0;
            return false;
        }

        if (_belowCrestSince == 0) _belowCrestSince = now;
        return now - _belowCrestSince >= CREST_CONFIRM_MS;
    }

    // Starts pushing once the pitch leaves the band and keeps pushing until it
    // is back inside half the band, so it doesn't chatter at the edge.
    int holdPower(float pitchDeg, const AutoParams &p) {
        float band = (float) p[P_HOLD_BAND];

        if (_push > 0 && pitchDeg < band / 2) _push = 0;
        if (_push < 0 && pitchDeg > -band / 2) _push = 0;

        if (pitchDeg > band) _push = 1;
        else if (pitchDeg < -band) _push = -1;

        if (_push > 0) return p[P_HOLD_FORWARD_POWER];
        if (_push < 0) return -p[P_HOLD_BACK_POWER];
        return 0;
    }

    static int clampPower(int v) { return v > 255 ? 255 : v < -255 ? -255 : v; }

    AutoState _state = AutoState::Idle;
    EndReason _lastEnd = EndReason::None;
    unsigned long _runStart = 0;
    unsigned long _stateStart = 0;
    unsigned long _belowCrestSince = 0;
    bool _untimed = false;
    int _push = 0;      // +1 pushing forward, -1 backing up, 0 braked
};


#endif //MONARCH_OF_THE_HILL_AUTOPILOT_H
