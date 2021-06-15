// Developed by @lucns

#include "SX126x.h"
#include "Arduino.h"

class Lora {
  private:
    SX126x sx;
    bool sleep, deviceOK;
  public:
    Lora();
    void initialize(int nss, int nrst, int busy);
    bool isDeviceOK();
	int getRssi();
    String read();
    bool write(String s);
    void powerUp();
    void powerDown();
	bool isSleeping();
};
