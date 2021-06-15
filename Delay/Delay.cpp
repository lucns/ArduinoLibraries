// Developed by @lucns

#include "Delay.h"
#include "Arduino.h"

Delay::Delay() {
}

void Delay::reset() {
  start = millis();
  locked = false;
}

void Delay::lock() {
  locked = true;
}

void Delay::setTime(unsigned long _time) {
  timeStipulated = _time;
  // reset();
}

bool Delay::gate() {
  if (millis() - start < timeStipulated || locked) return false;
  locked = true;
  return true;
}

void Delay::forceGate() {
  locked = false;
  start = millis() - timeStipulated + 1;
}

bool Delay::timeout() {
  return millis() - start >= timeStipulated;
}
