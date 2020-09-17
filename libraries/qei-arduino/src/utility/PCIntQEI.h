/*
 * PCIntQEI.h
 *
 * Created: 2017/03/19 23:24:32
 *  Author: Na-x4 <Na-x4@outlook.com>
 */


#ifndef PCINTQEI_H_
#define PCINTQEI_H_

#include <Arduino.h>

#ifdef ARDUINO_ARCH_AVR
#if defined(digitalPinToPCICRbit) && defined(digitalPinToPCICR) && defined(digitalPinToPCMSK)

#if defined(ATTINYX5) && defined(PCMSK)
#define NUMOF_QEICH 1
#else
#define NUMOF_QEICH 4
#endif

int pcint_addQEICh(uint8_t, uint8_t, uint8_t, uint8_t*, long*, uint8_t);
void pcint_removeQEICh(uint8_t, uint8_t, uint8_t, int);

#endif
#endif

#endif /* PCINTQEI_H_ */
