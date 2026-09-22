//
// Created by gabeo on 9/21/2026.
//

#include <Arduino.h>
#include "SerialDebugger.h"

SerialDebugger::SerialDebugger(Action forward, Action back, Action left, Action right)
    : _forward(forward), _back(back), _left(left), _right(right) {}

void SerialDebugger::readInput() {
    while (Serial.available()) {
        switch (tolower(Serial.read())) {
            case 'w': if (_forward) _forward(); break;
            case 's': if (_back)    _back();    break;
            case 'a': if (_left)    _left();    break;
            case 'd': if (_right)   _right();   break;
            default:  break; // newlines, junk, anything unbound
        }
    }
}
