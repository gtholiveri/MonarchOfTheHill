//
// Created by gabeo on 9/21/2026.
//

#include "MotorDebugger.h"

// Chassis sides. Motors 1/2 face one way, 3/4 the other - motorControl()
// already compensates, so a positive speed drives every motor forward.
static const uint8_t LEFT_FRONT  = 1;
static const uint8_t LEFT_REAR   = 2;
static const uint8_t RIGHT_FRONT = 3;
static const uint8_t RIGHT_REAR  = 4;

MotorDebugger::MotorDebugger(MotorFn motorControl, int speed)
    : _motorControl(motorControl),
      _speed(speed),
      _serial(
          [this] { drive( _speed,  _speed); },  // w - forward
          [this] { drive(-_speed, -_speed); },  // s - back
          [this] { drive(-_speed,  _speed); },  // a - spin left
          [this] { drive( _speed, -_speed); }   // d - spin right
      ) {}

void MotorDebugger::readInput() {
    _serial.readInput();
}

void MotorDebugger::stop() {
    drive(0, 0);
}

void MotorDebugger::drive(int leftSpeed, int rightSpeed) {
    if (!_motorControl) return;

    _motorControl(LEFT_FRONT,  leftSpeed);
    _motorControl(LEFT_REAR,   leftSpeed);
    _motorControl(RIGHT_FRONT, rightSpeed);
    _motorControl(RIGHT_REAR,  rightSpeed);
}
