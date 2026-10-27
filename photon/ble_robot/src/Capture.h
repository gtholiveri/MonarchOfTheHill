#ifndef MONARCH_OF_THE_HILL_CAPTURE_H
#define MONARCH_OF_THE_HILL_CAPTURE_H

#include <stdint.h>
#include <stdlib.h>

// ---------------------------------------------------------------------------
// High-rate capture: every estimator tick of a run, kept in memory.
//
// The live feed is ten rows a second, which is what the radio comfortably
// carries alongside driving. The parts of a run worth studying - the launch,
// the crest, the stop - each last a few tenths of a second, so with capture
// switched on the robot also keeps every estimator tick, 50 a second, and the
// dashboard fetches the whole log once the run is over.
//
// While waiting, the last couple of seconds are kept in a small ring, so a log
// opens with the robot still sitting at the start. A log stays until the next
// one begins, so a fetch that failed can simply be tried again.
//
// The log itself is too big for the Photon 2's static RAM, so it comes from
// the heap, once, at startup.
//
// Plain C++ with no Particle dependencies.
// ---------------------------------------------------------------------------

struct CaptureSample {
    uint32_t ms;                // millis()
    int16_t pitchFused;         // hundredths of a degree
    int16_t pitchAccel;         // from the accelerometer alone, same units
    int16_t roll;
    int16_t accel[3];           // thousandths of a g
    int16_t gyro[3];            // tenths of a degree per second, offset removed
    int16_t yaw;                // tenths of a degree
    int16_t motorA;             // duty applied, -255..255
    int16_t motorB;
    uint16_t packMv;
    uint16_t packRaw;
    int8_t autoLeft;            // autopilot output / 2
    int8_t autoRight;
    uint8_t stage;              // AutoState
    uint8_t flags;              // CAPTURE_TRUSTED, CAPTURE_DRIVEN
};

static const uint8_t CAPTURE_TRUSTED = 0x01;
static const uint8_t CAPTURE_DRIVEN = 0x02;

class CaptureLog {
public:
    static const int CAPACITY = 6000;       // 2 min at 50 a second, 216 kB
    static const int PREROLL = 100;         // 2 s

    // Returns false if the memory isn't there, and capture then stays off.
    bool allocate() {
        if (!_log) _log = (CaptureSample *) malloc(sizeof(CaptureSample) * CAPACITY);
        return _log != nullptr;
    }

    void enable(bool on) {
        _enabled = on && _log != nullptr;
        if (!_enabled) _capturing = false;      // a ready log is kept
    }

    bool enabled() const { return _enabled; }
    bool capturing() const { return _capturing; }
    bool ready() const { return _ready; }
    bool truncated() const { return _truncated; }
    uint16_t id() const { return _id; }
    int count() const { return _count; }
    const CaptureSample &at(int i) const { return _log[i]; }

    // Once per estimator tick.
    void add(const CaptureSample &s) {
        if (!_enabled) return;

        if (_capturing) {
            if (_count < CAPACITY) _log[_count++] = s;
            else _truncated = true;
            return;
        }

        _pre[_preHead] = s;
        _preHead = (_preHead + 1) % PREROLL;
        if (_preCount < PREROLL) _preCount++;
    }

    // Starts a new log with the pre-roll in front, replacing any earlier log.
    void begin() {
        if (!_enabled || _capturing) return;

        _id++;
        _count = 0;
        int oldest = (_preHead - _preCount + PREROLL) % PREROLL;
        for (int i = 0; i < _preCount; i++) _log[_count++] = _pre[(oldest + i) % PREROLL];
        _preCount = 0;
        _preHead = 0;

        _capturing = true;
        _ready = false;
        _truncated = false;
    }

    void finish() {
        if (!_capturing) return;
        _capturing = false;
        _ready = _count > 0;
    }

private:
    CaptureSample *_log = nullptr;
    CaptureSample _pre[PREROLL];
    int _preHead = 0;
    int _preCount = 0;
    int _count = 0;
    uint16_t _id = 0;
    bool _enabled = false;
    bool _capturing = false;
    bool _ready = false;
    bool _truncated = false;
};

#endif //MONARCH_OF_THE_HILL_CAPTURE_H
