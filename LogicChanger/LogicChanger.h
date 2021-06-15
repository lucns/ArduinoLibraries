// Developed by @lucns

class LogicChanger {
  private:
    int pin;
    bool enabled;
    unsigned long start, on, off;
  public:
    LogicChanger();
    void setParameters(int _pin, unsigned long _on, unsigned long _off);
    void setTimers(unsigned long _on, unsigned long _off);
    void compute();
    void reset();
	void forceEnabled();
	void disable();
	bool isEnabled();
};
