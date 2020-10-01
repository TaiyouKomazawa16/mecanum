#ifndef _MECANUM4WD_H_
#define _MECANUM4WD_H_


#include "Wheel.h"

class Mecanum4WD
{
public:
    Mecanum4WD(Wheel *FLeft, Wheel *RLeft, Wheel *RRight, Wheel *FRight, int wheel_span)
    {
        _wheel_span = wheel_span;
        _FLeft = FLeft;
        _RLeft = RLeft;
        _RRight = RRight;
        _FRight = FRight;
    }

    void move(double x_mps, double y_mps, float theta)
    {
        int x_mmps = 1000 * x_mps;
        int y_mmps = 1000 * y_mps;
        int fl = -(x_mmps - y_mmps - theta * _wheel_span);
        int rl = -(x_mmps + y_mmps - theta * _wheel_span);
        int rr = x_mmps - y_mmps + theta * _wheel_span;
        int fr = x_mmps + y_mmps + theta * _wheel_span;
        
        _FLeft->set_mmps(fl);
        _RLeft->set_mmps(rl);
        _RRight->set_mmps(rr);
        _FRight->set_mmps(fr);
    }

    void get_odom(double &x_m, double &y_m, double &theta) 
    {
        double fl = - _FLeft->get_mm();
        double rl = - _RLeft->get_mm();
        double rr = _RRight->get_mm();
        double fr = _FRight->get_mm();

      y_m = ((-fl + rl - rr + fr) / 4.0) / 1000.0;
	    x_m = ((fl + rl + rr + fr) / 4.0) / 1000.0;
	    theta = (-fl - rl + rr + fr) / (4.0 * _wheel_span); 
    }

    void reset()
    {
        _FLeft->reset();
        _RLeft->reset();
        _RRight->reset();
        _FRight->reset();
    }

private:
    Wheel *_FLeft, *_RLeft, *_RRight, *_FRight;
    int _wheel_span;
};

#endif
