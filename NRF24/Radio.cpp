// Developed by @lucns

#include "Radio.h"
#include "Arduino.h"
#include "RF24.h"

RF24 rf24l01;
byte addrs[][6] = {"node1", "node2"};

void Radio::enableIdleMode() {
  if (!idle) rf24l01.startListening();
  idle = true;
}

void Radio::disableIdleMode() {
  if (idle) rf24l01.stopListening();
  idle = false;
}

Radio::Radio() {
}

void Radio::begin(int ce, int csn, int mode) {
  rf24l01.begin(ce, csn);
  switch(mode) {
    case 1:
      rf24l01.openWritingPipe(addrs[0]);
      rf24l01.openReadingPipe(0, addrs[1]);
      break;
    case 2:
      rf24l01.openWritingPipe(addrs[1]);
      rf24l01.openReadingPipe(0, addrs[0]);
      break;
  }
  rf24l01.setPALevel(RF24_PA_MAX);
  rf24l01.setCRCLength(RF24_CRC_8);
  rf24l01.setDataRate(RF24_250KBPS);
  rf24l01.setAutoAck(false);
}

bool Radio::isPowered() {
  return rf24l01.isPowered();
}

void Radio::powerUp() {
  if (!isPowered()) rf24l01.powerUp();
}
void Radio::powerDown() {
  disableIdleMode();
  if (isPowered()) rf24l01.powerDown();
}

bool Radio::write(String s) {
  bool response = false;

  if (rf24l01.isPowered()) {
    if (s.length() > 0) {
      char c[32];
      s.toCharArray(c, 32);

      disableIdleMode();
      // delay(10);
      response = rf24l01.write(c, sizeof(c));
    }
    enableIdleMode();
  }
  return response;
}

String Radio::read() {
  String s;
  if (rf24l01.isPowered()) {
    enableIdleMode();
    if (rf24l01.available()) {
      char c[32];
      rf24l01.read(c, 32);
      s = String(c);
    }
  }
  return s;
}
