/*
 * MotorDriver.h
 *
 * Created: 2017/03/14 19:30:25
 *  Author: Na-x4 <Na-x4@outlook.com>
 */


#ifndef MOTORDRIVER_H_
#define MOTORDRIVER_H_

#include <stdint.h>

class MotorDriver {
    public:
        MotorDriver();
        virtual ~MotorDriver() {}
        void drive(int);
        void drive(bool, int);
        bool safeDrive(int);
        bool safeDrive(bool, int);
        void deadtimeDrive(int, int);
        void deadtimeDrive(int, bool, int);
        void brake(int);
        void brake();
        void free();
        
    protected:
        virtual void _drive(bool, uint8_t) = 0;
        virtual void _brake(uint8_t) = 0;
        virtual void _free() = 0;
        
    private:
        bool _prevDir;
};

#endif /* MOTORDRIVER_H_ */
