//
// Created by gabeo on 9/21/2026.
//

#ifndef MONARCH_OF_THE_HILL_SERIALDEBUGGER_H
#define MONARCH_OF_THE_HILL_SERIALDEBUGGER_H

#include <functional>

// ---------------------------------------------------------------------------
// Drive the car from a serial monitor with WASD.
//
// Hand the constructor four lambdas; call readInput() from loop(). Every
// recognised character sitting in the serial buffer fires its lambda once,
// anything else is dropped (so the newline a monitor appends is harmless).
//
// WASD rather than the arrow keys on purpose: an arrow key arrives as a
// three-byte escape sequence (ESC [ A), which would need a little state
// machine to decode. One key, one byte, one lambda.
//
// Serial.begin() is the caller's job - this class only reads.
// ---------------------------------------------------------------------------

class SerialDebugger {
public:
    using Action = std::function<void()>;

    SerialDebugger(Action forward, Action back, Action left, Action right);

    // Drains the serial buffer, firing a lambda per recognised key.
    void readInput();

private:
    Action _forward;
    Action _back;
    Action _left;
    Action _right;
};


#endif //MONARCH_OF_THE_HILL_SERIALDEBUGGER_H
