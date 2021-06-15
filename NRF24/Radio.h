// Developed by @lucns

#include "Arduino.h"

#define MODE_MASTER 1
#define MODE_SLAVE 2

class Radio {
  private:
    bool idle;
    void enableIdleMode();
    void disableIdleMode();
  public:
    Radio();
    void begin(int ce, int csn, int mode = MODE_MASTER);
    bool write(String s);
    String read();
    void powerUp();
    void powerDown();
    bool isPowered();
};
