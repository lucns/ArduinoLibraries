// Developed by @lucns

#include "Button.h"
#include "Arduino.h"

Button::Button() {
}

void Button::setPin(int _pin) {
  pin = _pin;
  pinMode(pin, INPUT_PULLUP);
}

void Button::setParameters(int _pin, int _clicks) {
  setPin(_pin);
  clicks = _clicks;
  clicksTime = clicks * 166UL;
}

bool Button::isPressed() {
  return !digitalRead(pin);
}

bool Button::getState() {
  bool pressed = isPressed();
  unsigned long m = millis();
  difference = m - changedTime;
  if (pressed != lastState && difference > 25UL) {
	changedTime = m;
	if (!pressed && lastState && !wasLongClick) {
	  wasClick = true;	
	  if (!clicksCount) firstClickTime = m;
	  if (m - firstClickTime < clicksTime) {
		clicksCount++;
		if (clicksCount == 2) {
		  wasClick = false;
		  wasDoubleClick = true;
		} else {
		  wasDoubleClick = false;
		}
	  } else {
		clicksCount = 1;
		firstClickTime = m;
	  }
	}
    wasLongClick = false;	
    fivePressed = false;	
    teenPressed = false;
	lastState = pressed;
	difference = 0;
  }
  
  return lastState;
}

bool Button::onClick() {
  bool pressed = getState();
  if (!pressed && wasClick) {
    wasClick = false;
    return true;
  }
  return false;
}

bool Button::onDoubleClick() {
  if (wasDoubleClick) {
    wasDoubleClick = false;
    return true;
  }
  return false;
}

bool Button::onMultipleClick() {
  if (clicksCount == clicks) {
	clicksCount = 0;
	return true;
  }
  return false;
}

bool Button::onLongClick() {
  bool pressed = getState();
  if (!wasLongClick && pressed && difference > 500UL) {
    wasLongClick = true;
    return true;
  }
  return false;
}

bool Button::onTeenSeconds() {
  bool pressed = getState();
  if (!teenPressed && pressed && difference > 10000UL)  {
	teenPressed = true;
	return true;
  }
  return false;
}

bool Button::onFiveSeconds() {
  bool pressed = getState();
  if (!fivePressed && pressed && difference > 5000UL)  {
	fivePressed = true;
	return true;
  }
  return false;
}
