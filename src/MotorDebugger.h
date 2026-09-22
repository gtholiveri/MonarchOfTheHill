//
// Created by gabeo on 9/21/2026.
//

#ifndef MONARCH_OF_THE_HILL_MOTORDEBUGGER_H
#define MONARCH_OF_THE_HILL_MOTORDEBUGGER_H

#include <Arduino.h>
#include "SerialDebugger.h"

// ---------------------------------------------------------------------------
// WASD driving, built on SerialDebugger.
//
// Hand the constructor the motor primitive - the same
// motorControl(motor, speed) signature the shield uses - and call readInput()
// from loop(). Each key sets all four motors and they stay there until the
// next key, so stop() is the brake.
//
// Side assignment follows the shield's mirrored-direction quirk: motors 1/2
// are one side of the chassis and 3/4 the other. If your car turns the wrong
// way, swap LEFT_* and RIGHT_* in the .cpp.
// ---------------------------------------------------------------------------

class MotorDebugger {
public:
    // motorControl: signed speed, -255..255, sign sets direction.
    using MotorFn = void (*)(uint8_t motor, int speed);

    explicit MotorDebugger(MotorFn motorControl, int speed = 255);

    void readInput();
    void stop();

private:
    void drive(int leftSpeed, int rightSpeed);

    MotorFn _motorControl;
    int _speed;
    SerialDebugger _serial;
};


#endif //MONARCH_OF_THE_HILL_MOTORDEBUGGER_H
