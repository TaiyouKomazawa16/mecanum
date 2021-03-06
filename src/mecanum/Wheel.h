#ifndef _WHEEL_H_
#define _WHEEL_H_

#include <math.h>

#include <Arduino.h>
#include <MotorDriver.h>
#include <SoftwareQEI.h>

#include "FastPID.h"

#define RADPS_RANGE 1000
//モータードライバのブートストラップを考慮
#define PWM_MARGIN 2

class Wheel
{
public:
    Wheel(MotorDriver *md, SoftwareQEI *qei, FastPID *pid, const int ppr, const int gear_ratio=1, const int round_mm = 1, bool dir_reverse=false)
    :   _rate(4*ppr*gear_ratio), _round_mm(round_mm), M_2PI(2*M_PI), _reverse(dir_reverse)
    {
        _md = md;
        _qei = qei;
        _pid = pid;

        _rps = 0;

        _pid->set_pi_d_mode(true);
        _pid->setOutputRange(-256+PWM_MARGIN, 255-PWM_MARGIN);

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
        _rps = diff/sec;
        return _rps;
    }

    double get_last_rps()
    {
        return _rps;
    }

    void set_rads(double rads)
    {
        int16_t pwm = _pid->step(rads * RADPS_RANGE, M_2PI*get_rps() * RADPS_RANGE);
        set_pwm(pwm);
    }

    void set_mmps(int mmps)
    {
        int feed_mmps = _round_mm * get_rps();
        set_pwm(_pid->step(mmps, feed_mmps));
    }

    inline void set_pwm(int16_t duty)
    {
        _pwm = duty * ((_reverse) ? -1 : 1);
        _md->drive(_pwm);
    }

    int16_t get_pwm()
    {
        return _pwm;
    }

    void reset()
    {
        _md->drive(0);
        _qei->write(0);
        _last_rev = 0;
        _pid->clear();
    }


private:
    const double M_2PI;

    MotorDriver *_md;
    SoftwareQEI *_qei;
    FastPID *_pid;
    const int _rate;
    const int _round_mm;

    double _last_rev;
    double _rps;
    int16_t _pwm;
    unsigned long _last_micros;

    const bool _reverse;
};

#endif
