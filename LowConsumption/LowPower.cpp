#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/power.h>
#include <avr/interrupt.h>

#include "LowPower.h"

#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega168P__)
void LowPowerClass::bodDisable() {
  unsigned char tempreg;
  __asm__ __volatile__(
    "in %[tempreg], %[mcucr]" "\n\t"
    "ori %[tempreg], %[bods_bodse]" "\n\t"
    "out %[mcucr], %[tempreg]" "\n\t"
    "andi %[tempreg], %[not_bodse]" "\n\t"
    "out %[mcucr], %[tempreg]"
    : [tempreg] "=&d" (tempreg)
    : [mcucr] "I" _SFR_IO_ADDR(MCUCR),
    [bods_bodse] "i" (_BV(BODS) | _BV(BODSE)),
    [not_bodse] "i" (~_BV(BODSE))
  );
}
#endif

void LowPowerClass::uartDisable() {
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega168P__)
  UCSR0B &= ~(_BV(RXEN0));
  UCSR0B &= ~(_BV(TXEN0));
#else
  UCSR1B &= ~(_BV(RXEN1));
  UCSR1B &= ~(_BV(TXEN1));
#endif
}

void LowPowerClass::uartEnable() {
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega168P__)
  UCSR0B |= _BV(RXEN0);
  UCSR0B |= _BV(TXEN0);
#else
  UCSR1B |= _BV(RXEN1);
  UCSR1B |= _BV(TXEN1);
#endif
}

void LowPowerClass::sleepForever() {
  ADCSRA &= ~(1 << ADEN); // adc off
  uartDisable();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  cli();
  sleep_enable();
#if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega168P__)
  bodDisable(); // Only Pico Power devices can change BOD settings through software
#endif
  sei();
  sleep_cpu(); // 328p Optboot 1Mhz Internal = < 1uA, 32u4 LilyPadUSB 8Mhz Internal = 133uA
  sleep_disable();
  sei();
  uartEnable();
  ADCSRA |= (1 << ADEN); // adc on

#if defined (__AVR_ATmega32U4__)
  power_usb_enable();
#endif
}

ISR (WDT_vect) {
  wdt_disable();
}

LowPowerClass LowPower;
