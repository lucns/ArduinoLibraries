// Developed by @lucns

#include "Arduino.h"
#include "String.h"
#include "PinChanger.h"

PinChanger::PinChanger() {
}

void PinChanger::compute() {
  if (millis() - startTime < times[index]) return;
  startTime = millis();
  index++;
  if (index == length) index = 0;
  digitalWrite(pin, !disabled ? LOW : HIGH);
  disabled = !disabled;
}

void PinChanger::setParameters(long* t, int l, int p) {
  length = l;
  times = new long[length];
  for(int i = 0; i < length; i++) {
    times[i] = t[i];
  }
  pin = p;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);

  startTime = millis();
  compute(); // apply parameters
}
