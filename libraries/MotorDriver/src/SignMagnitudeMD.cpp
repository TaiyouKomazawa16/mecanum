/*
 * SignMagnitudeMD.cpp
 *
 * Created: 2017/03/14 20:02:23
 *  Author: Na-x4 <Na-x4@outlook.com>
 */

#include "SignMagnitudeMD.h"

SignMagnitudeMD::SignMagnitudeMD(uint8_t dir, uint8_t pwm): _dirPin(dir), _pwmPin(pwm) {
    pinMode(_dirPin, OUTPUT);
    pinMode(_pwmPin, OUTPUT);
    
    _free();
}

void SignMagnitudeMD::_drive(bool dir, uint8_t speed) {
    digitalWrite(_dirPin, dir ? LOW : HIGH);
    analogWrite(_pwmPin, speed);
}

void SignMagnitudeMD::_brake(uint8_t) {
    digitalWrite(_dirPin, HIGH);
    digitalWrite(_pwmPin, LOW);
}

void SignMagnitudeMD::_free() {
    digitalWrite(_dirPin, LOW);
    digitalWrite(_pwmPin, LOW);
}
