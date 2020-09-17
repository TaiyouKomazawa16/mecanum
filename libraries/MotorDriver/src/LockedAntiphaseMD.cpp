/*
 * LockedAntiphaseMD.cpp
 *
 * Created: 2017/03/14 20:02:23
 *  Author: Na-x4 <Na-x4@outlook.com>
 */

#include "LockedAntiphaseMD.h"

LockedAntiphaseMD::LockedAntiphaseMD(uint8_t dirPin) : _dirPin(dirPin) {
    pinMode(_dirPin, OUTPUT);
    
    _free();
}

void LockedAntiphaseMD::_drive(bool dir, uint8_t speed) {
    analogWrite(_dirPin, (UINT8_MAX / 2) + ((dir ? speed : -speed) / 2));
}

void LockedAntiphaseMD::_brake(uint8_t) {
    _free();
}

void LockedAntiphaseMD::_free() {
    analogWrite(_dirPin, UINT8_MAX / 2);
}
