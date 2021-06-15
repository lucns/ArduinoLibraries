// Developed by @lucns

class Button {
  private:
    int pin, lastState;
    unsigned long changedTime, difference;
    bool wasLongClick, wasClick, wasDoubleClick;	
	bool fivePressed, teenPressed;
    void setPin(int _pin);
	bool getState();
	
	int clicks, clicksCount;
	unsigned long clicksTime, firstClickTime;
  public:
    Button();
    void setParameters(int _pin, int _clicks);
    bool isPressed();
    bool onClick();
    bool onDoubleClick();
	bool onMultipleClick();
    bool onLongClick();
    bool onFiveSeconds();
    bool onTeenSeconds();
};
