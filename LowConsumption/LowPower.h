#ifndef LowPower_h
#define LowPower_h

#include "Arduino.h"

class LowPowerClass {
  private:
    void uartDisable();
    void uartEnable();
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega168P__)
    void bodDisable();
#endif
  public:
    void sleepForever();
};

extern LowPowerClass LowPower;
#endif
