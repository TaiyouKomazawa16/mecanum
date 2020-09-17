/*
 * LockedAntiphaseMD.h
 *
 * Created: 2017/03/14 20:02:36
 *  Author: Na-x4 <Na-x4@outlook.com>
 */


#ifndef LOCKEDANTIPHASEMD_H_
#define LOCKEDANTIPHASEMD_H_

#include "MotorDriver.h"

#include <Arduino.h>

class LockedAntiphaseMD: public MotorDriver {
    public:
        LockedAntiphaseMD(uint8_t);
        
    protected:
        virtual void _drive(bool, uint8_t) override;
        virtual void _brake(uint8_t) override;
        virtual void _free() override;
        
    private:
        uint8_t _dirPin;
};

#endif /* LOCKEDANTIPHASEMD_H_ */
