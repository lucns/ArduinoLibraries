// Developed by @lucns

#include "Fade.h"
#include "Arduino.h"

Fade::Fade() {
}

Fade::Fade(int _pin, long _time) {
  pin = _pin;
  interval = (_time * 1000) / resolution;
  if (interval < 1000) interval = 1000;
  pinMode(pin, OUTPUT);
}

void Fade::setParameters(int _pin, long _time, int _resolution) {
  pin = _pin;
  resolution = _resolution;
  interval = (_time * 1000) / resolution;
  // if (interval < 1000) interval = 1000;
  pinMode(pin, OUTPUT);
}

void Fade::compute() {
  if (level == resolution) mainInterval = interval / resolution;
  else if (level == 0) mainInterval = interval;
  else mainInterval = (interval / resolution) * (resolution - level);
  if (mainInterval == 0) mainInterval = 1;

  if (micros() - startTime < mainInterval) return;
  startTime = micros();
  if (fadeIn && level < resolution && millis() - pausedAt >= pauseTime) level++;
  else if (level > 0) level--;

  if (level == resolution) fadeIn = false;
  if (level == 0 && !fadeIn) {
    fadeIn = true;
    pausedAt = millis();
  }
  analogWrite(pin, level);
}

void Fade::disable() {
  level = 0;
  digitalWrite(pin, LOW);
}

void Fade::enable() {
  level = resolution;
  digitalWrite(pin, HIGH);
}

void Fade::insertDelay(long _delay) {
  pauseTime = _delay;
}
