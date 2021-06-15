// Developed by @lucns

#include "Arduino.h"
#include "String.h"
#include "Pulsator.h"

Pulsator::Pulsator() {
}

void Pulsator::setPin(int p, int d) {
  duration = d;
  pin = p;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void Pulsator::pulsate(int r) {
  timed = false;
  repetition = (r * 2) - 1;
  state = true;
  digitalWrite(pin, state);
  change = millis();
  count = 0;
}

void Pulsator::vibrate() {
  timed = false;
  digitalWrite(pin, HIGH);
}

void Pulsator::vibrateTimed(int d) {
  timed = true;
  timedDuration = d;
  timedStart = millis();
  digitalWrite(pin, HIGH);
}

void Pulsator::disable() {
  timed = false;
  state = false;
  count = repetition;
  digitalWrite(pin, LOW);
}

void Pulsator::compute() {
  if (timed) {
    if (millis() - timedStart >= timedDuration) {
      state = false;
	  digitalWrite(pin, LOW);
	}
    return;
  }
  if (change == 0 || millis() - change < duration) return;
  if (count == repetition) {
    digitalWrite(pin, LOW);
  } else {
    count++;
    state = !state;
    digitalWrite(pin, state);
    change = millis();
  }
}
