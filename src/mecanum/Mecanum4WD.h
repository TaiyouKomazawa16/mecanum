#ifndef _MECANUM4WD_H_
#define _MECANUM4WD_H_

#include "Wheel.h"

class Mecanum4WD
{
public:
    Mecanum4WD(Wheel *FLeft, Wheel *RLeft, Wheel *RRight, Wheel *FRight, unsigned int wheel_span_x, unsigned int wheel_span_y)
    {
        _ws_x = wheel_span_x/2;
        _ws_y = wheel_span_y/2;
        _FLeft = FLeft;
        _RLeft = RLeft;
        _RRight = RRight;
        _FRight = FRight;
        _prev_x = 0;
        _prev_y = 0;
        _prev_time = millis();
        delay(1);
    }

    void move(double x_mps, double y_mps, float theta)
    {
        int x_mmps = 1000 * x_mps;
        int y_mmps = 1000 * y_mps;
        int rot = theta * (_ws_x+_ws_y);
        int fl = -(x_mmps - y_mmps - rot);
        int rl = -(x_mmps + y_mmps - rot);
        int rr = x_mmps - y_mmps + rot;
        int fr = x_mmps + y_mmps + rot;
        
        _FLeft->set_mmps(fl);
        _RLeft->set_mmps(rl);
        _RRight->set_mmps(rr);
        _FRight->set_mmps(fr);
    }

    void get_odom(double &x_vel, double &y_vel, double &theta) 
    {
        double fl = - _FLeft->get_mm();
        double rl = - _RLeft->get_mm();
        double rr = _RRight->get_mm();
        double fr = _FRight->get_mm();

        double x = ((fl + rl + rr + fr) / 4.0);
        double y = ((-fl + rl - rr + fr) / 4.0);
        double dt = (millis() - _prev_time);
        _prev_time = millis();
        
        double div_x = (x - _prev_x) / dt; //[mm] / [ms] = [m/s]
        double div_y = (y - _prev_y) / dt; //[mm] / [ms] = [m/s]

        theta = (-fl - rl + rr + fr) / (4.0 * (_ws_x+_ws_y));
        x_vel = div_x*cos(theta) - div_y*sin(theta);
        y_vel = div_x*sin(theta) + div_y*cos(theta);

        _prev_x = x;
        _prev_y = y;
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
    double _prev_x, _prev_y;
    long _prev_time;
    int _ws_x, _ws_y;
    double _w_dir;
};

#endif
