// Developed by @lucns

class Fade {
  private:
    long interval, mainInterval, startTime, pauseTime, pausedAt;
    int pin, level, resolution;
    bool fadeIn;
  public:
    Fade();
    Fade(int _pin, long _time);
    void setParameters(int _pin, long _time, int _resolution = 255);
    void compute();
    void disable();
    void enable();
    void insertDelay(long _delay);
};
