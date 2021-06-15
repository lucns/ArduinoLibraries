// Developed by @lucns

#include "Arduino.h"
#include "String.h"
#include "LogicChanger.h"

LogicChanger::LogicChanger() {
}

void LogicChanger::compute() {
  if (millis() - start < (enabled ? on : off)) return;
  start = millis();
  enabled = !enabled;
  digitalWrite(pin, enabled);
}

void LogicChanger::setParameters(int _pin, unsigned long _on, unsigned long _off) {
  pin = _pin;
  on = _on;
  off = _off;
  enabled = false;
  pinMode(pin, OUTPUT);
  digitalWrite(pin, LOW);
}

void LogicChanger::setTimers(unsigned long _on, unsigned long _off) {
  on = _on;
  off = _off;
}

void LogicChanger::reset() {
  start = millis();
}

void LogicChanger::forceEnabled() {
  enabled = true;
  start = millis();
  digitalWrite(pin, HIGH);
}

void LogicChanger::disable() {
  enabled = false;
  digitalWrite(pin, LOW);
}	

bool LogicChanger::isEnabled() {
  return enabled;
}