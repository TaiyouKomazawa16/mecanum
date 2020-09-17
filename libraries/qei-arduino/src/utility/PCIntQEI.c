/*
 * PCIntQEI.c
 *
 * Created: 2017/03/19 23:24:17
 *  Author: Na-x4 <Na-x4@outlook.com>
 */

#include "PCIntQEI.h"

#ifdef ARDUINO_ARCH_AVR
#if defined(digitalPinToPCICRbit) && defined(digitalPinToPCICR) && defined(digitalPinToPCMSK)

#include <avr/io.h>

typedef struct {
    uint8_t isEnabled;
    uint8_t pinZEnable;
    uint8_t pinAMask;
    uint8_t pinBMask;
    uint8_t pinZMask;
    uint8_t prevPinAState;
    uint8_t prevPinBState;
    uint8_t prevPinZState;
    uint8_t* toggleIntPinZ;
    long* count;
} QEIChInf_t;

#ifdef PCMSK
QEIChInf_t qeiChInf_pcint_[NUMOF_QEICH] = {};
#endif

#ifdef PCMSK0
QEIChInf_t qeiChInf_pcint0_[NUMOF_QEICH] = {};
#endif

#ifdef PCMSK1
QEIChInf_t qeiChInf_pcint1_[NUMOF_QEICH] = {};
#endif

#ifdef PCMSK2
QEIChInf_t qeiChInf_pcint2_[NUMOF_QEICH] = {};
#endif

int pcint_addQEICh(uint8_t pina, uint8_t pinb, uint8_t pinz, uint8_t *toggleIntPinZ, long *count, uint8_t pinZEnable) {
    int i = 0;
    QEIChInf_t *qeiChInf = NULL;
    

#if defined(ATTINYX5) && defined(PCMSK)
    qeiChInf = qeiChInf_pcint_;
    PCMSK = _BV(pina) | _BV(pinb);
    if (pinZEnable)
        PCMSK |= _BV(pinz); 

    GIMSK |= _BV(PCIE);
    GIFR |= _BV(PCIF);
#else
    int pinaPCICRbit = digitalPinToPCICRbit(pina);
    if(pinaPCICRbit == digitalPinToPCICRbit(pinb) && ((pinZEnable) ? pinaPCICRbit == digitalPinToPCICRbit(pinz) : 1)) {
        switch(pinaPCICRbit) {
#ifdef PCMSK0
        case 0:
            qeiChInf = qeiChInf_pcint0_;
            break;
#endif
            
#ifdef PCMSK1
        case 1:
            qeiChInf = qeiChInf_pcint1_;
            break;
#endif
            
#ifdef PCMSK2
        case 2:
            qeiChInf = qeiChInf_pcint2_;
            break;
#endif
            
        default:
            return -1;
        }
        
        //*digitalPinToPCICR(pina) |= _BV(pinaPCICRbit);
        PCICR |= _BV(pinaPCICRbit);
    } else {
        return -1;
    }
#endif

    for(i = 0; i < NUMOF_QEICH; i++) {
        if(!qeiChInf[i].isEnabled) {
            qeiChInf[i].pinAMask = digitalPinToBitMask(pina);
            qeiChInf[i].pinBMask = digitalPinToBitMask(pinb);
            qeiChInf[i].count = count;

            *digitalPinToPCMSK(pina) |= qeiChInf[i].pinAMask;
            *digitalPinToPCMSK(pinb) |= qeiChInf[i].pinBMask;
            
            qeiChInf[i].isEnabled = 1;
            qeiChInf[i].pinZEnable = pinZEnable;
            if(pinZEnable) {
                qeiChInf[i].pinZMask = digitalPinToBitMask(pinz);

                *digitalPinToPCMSK(pinz) |= qeiChInf[i].pinZMask;

                qeiChInf[i].toggleIntPinZ = toggleIntPinZ;
            }
            return i;
        }
    }
    return -1;
}

void pcint_removeQEICh(uint8_t pina, uint8_t pinb, uint8_t pinz, int ch) {
    QEIChInf_t *qeiChInf;

#if defined(ATTINYX5) && defined(PCMSK)
    qeiChInf = qeiChInf_pcint_;
    PCMSK &= ~(_BV(pina) | _BV(pinb));
    if (qeiChInf[ch].pinZEnable)
        PCMSK &= ~_BV(pinz);
    
#else
    int pinaPCICRbit = digitalPinToPCICRbit(pina);
    if(pinaPCICRbit != digitalPinToPCICRbit(pinb)) {
        return;
    }
    if((qeiChInf[ch].pinZEnable) ? (pinaPCICRbit != digitalPinToPCICRbit(pinz)) : 0) {
        return;
    }

    switch (pinaPCICRbit) {
#ifdef PCMSK0
    case 0:
        qeiChInf = qeiChInf_pcint0_;
        break;
#endif
        
#ifdef PCMSK1
    case 1:
        qeiChInf = qeiChInf_pcint1_;
        break;
#endif
        
#ifdef PCMSK2
    case 2:
        qeiChInf = qeiChInf_pcint2_;
        break;
#endif
        
    default:
        return;
    }
#endif
    qeiChInf[ch].pinZEnable = 0;
    qeiChInf[ch].isEnabled = 0;
}

inline static void qeiCalc(QEIChInf_t *qeiChInf, volatile uint8_t *port) {
    int i = NUMOF_QEICH;
    uint8_t currentPinAState, currentPinBState, prevPinAState, prevPinBState;
    
    while(i--) {
        QEIChInf_t *qeiChInf_i = &qeiChInf[i];
        if(qeiChInf[i].isEnabled) {
            currentPinAState = *port & (qeiChInf_i->pinAMask);
            currentPinBState = *port & (qeiChInf_i->pinBMask);
            prevPinAState = qeiChInf_i->prevPinAState;
            prevPinBState = qeiChInf_i->prevPinBState;
            
            if(currentPinAState ^ prevPinAState || currentPinBState ^ prevPinBState) {
                if((currentPinAState && !prevPinBState) || (!currentPinAState && prevPinBState)) {
                    (*(qeiChInf_i->count))++;
                } else {
                    (*(qeiChInf_i->count))--;
                }
                qeiChInf_i->prevPinAState = currentPinAState;
                qeiChInf_i->prevPinBState = currentPinBState;
            }

            if (qeiChInf[i].pinZEnable && (*port & (qeiChInf_i->pinZMask))) {
                    *(qeiChInf_i->toggleIntPinZ) = ~(*(qeiChInf_i->toggleIntPinZ));
            }
        }
    }
}

#ifdef PCMSK
ISR(PCINT0_vect) {
    qeiCalc(qeiChInf_pcint_, &PINB);
}
#endif

#ifdef PCMSK0
ISR(PCINT0_vect) {
    qeiCalc(qeiChInf_pcint0_, &PINB);
}
#endif

#ifdef PCMSK1
ISR(PCINT1_vect) {
    qeiCalc(qeiChInf_pcint1_, &PINC);
}
#endif

#ifdef PCMSK2
ISR(PCINT2_vect) {
    qeiCalc(qeiChInf_pcint2_, &PIND);
}
#endif


#endif
#endif
