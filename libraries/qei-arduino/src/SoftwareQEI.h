/*
 * SoftwareQEI.h
 *
 * Created: 2017/03/19 11:37:45
 *  Author: Na-x4 <Na-x4@outlook.com>
 */


#ifndef SOFTWAREQEI_H_
#define SOFTWAREQEI_H_

#include <Arduino.h>
#include "QEI.h"

#ifdef ARDUINO_ARCH_AVR
#if defined(digitalPinToPCICRbit) && defined(digitalPinToPCICR) && defined(digitalPinToPCMSK)

class SoftwareQEI : public QEI {
    public:
        SoftwareQEI(uint8_t, uint8_t);
        SoftwareQEI(uint8_t, uint8_t, uint8_t);
        ~SoftwareQEI();
        virtual long read() override;
        virtual void write(long) override;
        virtual bool get_z() override;
    private : 
        uint8_t pina_;
        uint8_t pinb_;
        uint8_t pinz_;
        long count_;
        uint8_t toggle_z_;
        int qeiCh_;
};

#endif
#endif

#endif /* SOFTWAREQEI_H_ */
