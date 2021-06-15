// Developed by @lucns

#include "Buzzer.h"
#include "Arduino.h"

Buzzer::Buzzer() {
  gate.setTime(100);
  timeout.setTime(50);
}

void Buzzer::setPin(int _pin) {
  pin = _pin;
  if (!makeTone) pinMode(pin, OUTPUT);
}

void Buzzer::beep() {
  if (makeTone) {
    tone(pin, defaultFrequency, 50);
  } else {
	timeout.reset();
	digitalWrite(pin, HIGH);
  }
}

void Buzzer::beepRepeatedly(unsigned int beeps) {
  targetCount = beeps;
  count = 1;
  gate.reset();
  if (makeTone) {
	tone(pin, defaultFrequency, 50);
  } else {
	timeout.reset();
	digitalWrite(pin, HIGH);
  }
}

void Buzzer::longBeep() {
  if (makeTone) tone(pin, defaultFrequency, defaultDuration);
  else digitalWrite(pin, HIGH);
}

void Buzzer::compute() {
  if (timeout.gate()) digitalWrite(pin, LOW);
  if (gate.gate() && count != targetCount) {
	count++;
	gate.reset();
	if (makeTone) {
	  tone(pin, defaultFrequency, 50);
	} else {
	  timeout.reset();
	  digitalWrite(pin, HIGH);
	}
  }
}

void Buzzer::enableTone(bool enable) {
  makeTone = enable;
  if (!makeTone &&  pin) pinMode(pin, OUTPUT);
}

void Buzzer::disable() {
  if (makeTone) noTone(pin);
  else digitalWrite(pin, LOW);
}
