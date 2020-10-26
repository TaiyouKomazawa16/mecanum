#ifndef _WHEEL_H_
#define _WHEEL_H_

#include <math.h>

#include <Arduino.h>
#include <MotorDriver.h>
#include <SoftwareQEI.h>

#include "FastPID.h"

#define RADPS_RANGE 1000

class Wheel
{
public:
    Wheel(MotorDriver *md, SoftwareQEI *qei, FastPID *pid, const int ppr, const int gear_ratio=1, const int round_mm = 1)
    :   _rate(ppr * gear_ratio * 4), _round_mm(round_mm)
    {
        _md = md;
        _qei = qei;
        _pid = pid;

        _pid->setOutputConfig(9, true);

        _last_rev = get_revolution();
        _last_micros = micros();
    }

    inline double get_revolution()
    {
        return (double)_qei->read() / _rate;
    }

    inline double get_mm()
    {
        return get_revolution() * _round_mm;
    }

    inline double get_rad()
    {
        return get_revolution() * M_2PI;
    }

    double get_rps()
    {
        double sec = (micros() - _last_micros) / 1.0e6;
        while(sec == 0){ 
          sec = (micros() - _last_micros) / 1.0e6;
          delay(1);
        }
        double diff = get_revolution() - _last_rev;
        _last_rev = get_revolution();
        _last_micros = micros();
        return diff/sec;
    }

    void set_rads(double rads)
    {
        int16_t pwm = _pid->step(rads * RADPS_RANGE, M_2PI*get_rps() * RADPS_RANGE);
        _md->drive(pwm);
    }

    void set_mmps(int mmps)
    {
        int feed_mmps = _round_mm * get_rps();
        int16_t pwm = _pid->step(mmps, feed_mmps);
        _md->drive(pwm);
    }

    void reset()
    {
        _md->drive(0);
        _qei->write(0);
        _last_rev = 0;
        _pid->clear();
    }

    
private:
    const static double M_2PI;

    MotorDriver *_md;
    SoftwareQEI *_qei;
    FastPID *_pid;
    const int _rate;
    const int _round_mm;

    double _last_rev;
    unsigned long _last_micros;
};

const static double Wheel::M_2PI = 2*M_PI;

#endif
