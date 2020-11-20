#include <PlainSerial.h>
#include <msg/PlaneTwist.h>
#include <msg/Bools.h>

#include <MotorDriver.h>
#include <SignMagnitudeMD.h>
#include <SoftwareQEI.h>

#include "FastPID.h"

#include "Wheel.h"
#include "Mecanum4WD.h"

#include "WheelsMsg.h"

PlainSerial uart(&Serial);
PlaneTwist  cmd;
PlaneTwist  odom;
WheelsMsg   wheel_vel;
Bools       bools;

MotorDriver *md[4];
SoftwareQEI *qei[4];
FastPID *pid[4];

Wheel *wheel[4];

Mecanum4WD *mv;

/*
    Mecanum wheels

      WHEEL_SPAN_A
    |------------|
        Front         ---
    0 |--------| 3     |
      |\      /|       |
  Left|        |Right  | WHEEL_SPAN_B
      |/      \|       |
    1 |--------| 2     |
    x   Rear          ---
    ↑
  y←
 (yaw=CCW)
*/

/*device parameters-----*/

#define PPR           12  //Pulse Per Revolution
#define GEAR_RATIO    64  //1/GEAR_RATIO (wheels/motors)
#define WHEEL_CIR     314 //Wheel circumference[mm]
#define WHEEL_SPAN_A  280 //[mm]
#define WHEEL_SPAN_B  250 //[mm]

/*---------------------*/

void init_motor()
{
  TCCR1B = TCCR1B & 0xf8 | 0x01; // Pin9,Pin10 PWM 31250Hz
  TCCR2B = TCCR2B & 0xf8 | 0x01; // Pin3,Pin11 PWM 31250Hz
                            /*Dir PWM*/
  md[0] = new SignMagnitudeMD(2,  3);
  md[1] = new SignMagnitudeMD(12, 11);
  md[2] = new SignMagnitudeMD(8,  9);
  md[3] = new SignMagnitudeMD(7,  10);
}

void init_qei()
{
                          /*A   B*/
  qei[0] = new SoftwareQEI(4,   5);
  qei[1] = new SoftwareQEI(14, 15);
  qei[2] = new SoftwareQEI(16, 17);
  qei[3] = new SoftwareQEI(18, 19);
}

void init_pid()
{
                        /*kp    ki    kd   ctrl_freq*/
  pid[0] = new FastPID  (1.1, 0.5, 0.0001, 50 /*Hz*/, true);
  pid[1] = new FastPID  (1.1, 0.5, 0.0001, 50 /*Hz*/, true);
  pid[2] = new FastPID  (1.1, 0.5, 0.0001, 50 /*Hz*/, true);
  pid[3] = new FastPID  (1.1, 0.5, 0.0001, 50 /*Hz*/, true);
}

void init_wheels()
{
  wheel[0] = new Wheel(md[0], qei[0], pid[0], PPR, GEAR_RATIO, WHEEL_CIR);
  wheel[1] = new Wheel(md[1], qei[1], pid[1], PPR, GEAR_RATIO, WHEEL_CIR);
  wheel[2] = new Wheel(md[2], qei[2], pid[2], PPR, GEAR_RATIO, WHEEL_CIR);
  wheel[3] = new Wheel(md[3], qei[3], pid[3], PPR, GEAR_RATIO, WHEEL_CIR);
  mv = new Mecanum4WD(wheel[0], wheel[1], wheel[2], wheel[3], WHEEL_SPAN_B, WHEEL_SPAN_A);
}

double odom_y = 0, odom_x = 0, odom_yaw = 0;

void setup()
{
  init_motor();
  init_qei();
  init_pid();
  init_wheels();

  Serial.begin(9600);
  uart.add_frame(&cmd);
  uart.add_frame(&bools);
  uart.wait_host("Movement");
}

void loop()
{
  //odom:   odometry data [m/s](yaw[rad])
  mv->get_odom(odom_x, odom_y, odom_yaw);
  odom.x(odom_x);
  odom.y(odom_y);
  odom.yaw(odom_yaw);
  uart.write(0, &odom);
  
/*  wheel_vel.wheel.fl = wheel[0]->get_last_rps();
  wheel_vel.wheel.rl = wheel[1]->get_last_rps();
  wheel_vel.wheel.rr = wheel[2]->get_last_rps();
  wheel_vel.wheel.fr = wheel[3]->get_last_rps();

  uart.write(2, &wheel_vel);
*/
  //bools:  received commands for system
  //  |-bit 0:  reset_command
  //  |-bit 1~8:unassigned
  if (uart.read() == 0) {
    if (bools.get(0)) {
      mv->reset();
      delay(500);
      odom.set(0, 0, 0);
      bools.set(false, 0);
    }
  }

  //cmd:  received command velocity data [m/s](yaw[rad/s])
  mv->move(cmd.x(), cmd.y(), cmd.yaw());
  delayMicroseconds(500);
}
