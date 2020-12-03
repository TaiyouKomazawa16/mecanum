#include "PCIntQEI.h"

#ifdef ARDUINO_ARCH_STM32

#define NUMOF_QEICH 4

typedef struct {
    uint8_t isEnabled;
    uint8_t pinZEnable;
    uint8_t pinA;
    uint8_t pinB;
    uint8_t pinZ;
    uint8_t prevPinAState;
    uint8_t prevPinBState;
    uint8_t prevPinZState;
    uint8_t* toggleIntPinZ;
    long* count;
} QEIChInf_t;

QEIChInf_t qeiChInf_pcint_[NUMOF_QEICH] = {};

void QEI_EVENT_ISR(void)
{
    int i = NUMOF_QEICH;
    uint8_t currentPinAState, currentPinBState, prevPinAState, prevPinBState;
    while(i--) {
        QEIChInf_t *qeiChInf_i = &qeiChInf_pcint_[i];
        if(qeiChInf_pcint_[i].isEnabled) {
            currentPinAState = digitalRead(qeiChInf_i->pinA);
            currentPinBState = digitalRead(qeiChInf_i->pinB);
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
        }
    }
}

void QEI_EVENT_Z_ISR(void)
{
    int i = NUMOF_QEICH;
    while(i--) {
        QEIChInf_t *qeiChInf_i = &qeiChInf_pcint_[i];
        if (qeiChInf_i->pinZEnable) {
                *(qeiChInf_i->toggleIntPinZ) = ~(*(qeiChInf_i->toggleIntPinZ));
        }
    }
    
}

int pcint_addQEICh(uint8_t pina, uint8_t pinb, uint8_t pinz, uint8_t *toggleIntPinZ, long *count, uint8_t pinZEnable)
{
    for(int i = 0; i < NUMOF_QEICH; i++) {
        if(!qeiChInf_pcint_[i].isEnabled) {
            qeiChInf_pcint_[i].count = count;
            qeiChInf_pcint_[i].pinA = pina;
            qeiChInf_pcint_[i].pinB = pinb;

            attachInterrupt(pina, QEI_EVENT_ISR, CHANGE);
            attachInterrupt(pinb, QEI_EVENT_ISR, CHANGE);
            
            qeiChInf_pcint_[i].isEnabled = 1;
            qeiChInf_pcint_[i].pinZEnable = pinZEnable;
            if(pinZEnable) {
                attachInterrupt(pinz, QEI_EVENT_Z_ISR, CHANGE);
                qeiChInf_pcint_[i].toggleIntPinZ = toggleIntPinZ;
            }
            return i;
        }
    }
}
void pcint_removeQEICh(uint8_t pina, uint8_t pinb, uint8_t pinz, int ch)
{
    detachInterrupt(pina);
    detachInterrupt(pinb);
    detachInterrupt(pinz);

    qeiChInf_pcint_[ch].pinZEnable = 0;
    qeiChInf_pcint_[ch].isEnabled = 0;
}

#endif 
