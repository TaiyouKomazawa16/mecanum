/*
 * SignMagnitudeMD.h
 *
 * Created: 2017/03/14 20:02:36
 *  Author: Na-x4 <Na-x4@outlook.com>
 */


#ifndef SIGNMAGNITUDEMD_H_
#define SIGNMAGNITUDEMD_H_

#include "MotorDriver.h"

#include <Arduino.h>

class SignMagnitudeMD: public MotorDriver {
    public:
        SignMagnitudeMD(uint8_t, uint8_t);
        
    protected:
        virtual void _drive(bool, uint8_t) override;
        virtual void _brake(uint8_t) override;
        virtual void _free() override;
        
    private:
        uint8_t _dirPin;
        uint8_t _pwmPin;
};

#endif /* SIGNMAGNITUDEMD_H_ */
