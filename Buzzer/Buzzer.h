// Developed by @lucns
#include <Delay.h>

class Buzzer {
  private:
	Delay gate, timeout;
	bool makeTone;
	const unsigned int defaultFrequency = 1000;
	const unsigned long defaultDuration = 250UL;
	int pin;
	int count, targetCount;
  public:
    Buzzer();
    void setPin(int _pin);
    void beep();
    void beepRepeatedly(unsigned int beeps = 2);
	void longBeep();
    void enableTone(bool enable = true);
    void disable();
	void compute();
};
