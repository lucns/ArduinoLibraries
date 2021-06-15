// Developed by @lucns

class PinChanger {
  private:
    int pin, index, length;
    bool disabled;
    long startTime;
    long* times;
  public:
    PinChanger();
    void setParameters(long* t, int l, int p);
    void compute();
};
