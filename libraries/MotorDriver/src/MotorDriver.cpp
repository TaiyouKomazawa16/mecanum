/*
 * MotorDriver.cpp
 *
 * Created: 2017/03/14 19:38:53
 *  Author: Na-x4 <Na-x4@outlook.com>
 */

#include "MotorDriver.h"

#include <Arduino.h>

MotorDriver::MotorDriver(): _prevDir(true) {}

void MotorDriver::drive(int speed) {
    drive((bool)(speed >= 0), ((speed < 0) ? -speed : speed));
}

void MotorDriver::drive(bool dir, int speed) {
    _drive(dir, speed);
}

bool MotorDriver::safeDrive(int speed) {
    return safeDrive((bool)(speed >= 0), ((speed < 0) ? -speed : speed));
}

bool MotorDriver::safeDrive(bool dir, int speed) {
    if(_prevDir == dir) {
        drive(dir, speed);
        return true;
    } else {
        _prevDir = dir;
        free();
        return false;
    }
}

void MotorDriver::deadtimeDrive(int interval, int speed) {
    deadtimeDrive(interval, (bool)(speed >= 0), ((speed < 0) ? -speed : speed));
}

void MotorDriver::deadtimeDrive(int interval, bool dir, int speed) {
    if(!safeDrive(dir, speed)) {
        delay(interval);
        safeDrive(dir, speed);
    }
}

void MotorDriver::brake(int strength) {
    _brake(strength);
}

void MotorDriver::brake() {
    _brake(UINT8_MAX);
}

void MotorDriver::free() {
    _free();
}
