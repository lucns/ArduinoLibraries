// Developed by @lucns

class Delay {
  private:
    unsigned long start;
    unsigned long timeStipulated;
    bool locked;
  public:
    Delay();
    void reset();
	void lock();
    void setTime(unsigned long _time);
    bool gate();
	void forceGate();
	bool timeout();
};
