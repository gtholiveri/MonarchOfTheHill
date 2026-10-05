#ifndef MONARCH_DRIVETRAIN_H
#define MONARCH_DRIVETRAIN_H

#include <Arduino.h>

// ---------------------------------------------------------------------------
// Left/right drive for a skid-steer (tracked) robot on the TB6612FNG board
// (pins in config.h). The strategy only ever says "left side this fast, right
// side that fast"; everything board-specific stays in DriveTrain.cpp.
//
// Speeds are -255..255, positive = the robot moves forward. A speed of 0 on
// both sides holds the robot still (short brake where the board supports it,
// which also resists being shoved while holding the mesa).
// ---------------------------------------------------------------------------
class DriveTrain {
public:
    // Sets the pins up with both motors held in brake, then enables the driver.
    bool begin();

    void drive(int left, int right);
    void stop() { drive(0, 0); }

    // What the last drive() call asked for, for the bench printout.
    int lastLeft() const { return _left; }
    int lastRight() const { return _right; }

private:
    int _left = 0;
    int _right = 0;
};

#endif // MONARCH_DRIVETRAIN_H
