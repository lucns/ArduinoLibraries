// Developed by @lucns

class Pulsator {
  private:
    int pin, count, repetition, duration, timedDuration;
    unsigned long change, timedStart;
    bool state, timed;
  public:
    Pulsator();
    void setPin(int p, int d = 25);
    void pulsate(int r);
    void vibrateTimed(int d);
    void vibrate();
    void disable();
    void compute();
};
