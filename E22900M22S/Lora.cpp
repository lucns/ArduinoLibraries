// Developed by @lucns

#include "Lora.h"
#include "Arduino.h"

#define RF_FREQUENCY 868000000  // Hz
#define TX_OUTPUT_POWER 22    // dBm

Lora::Lora() {
}

void Lora::initialize(int nss, int nrst, int busy) {
  sx.begin(nss, nrst, busy);
  deviceOK = sx.configure(SX126X_PACKET_TYPE_LORA, RF_FREQUENCY, TX_OUTPUT_POWER, LORA_SPREADING_FACTOR_5, SX126X_LORA_BW_500_0, SX126X_LORA_CR_4_5, LORA_PREAMBLE_DEFAULT, LORA_SYSTEM_MAKE_PAYLOAD_LENGTH, false, false); //LoRa or FSK, FSK currently not supported, crcOn  false, invertIrq = false
  //Serial.println(deviceOK ? "Lora Device OK" : "Device not found or bad configuration");
}

bool Lora::isDeviceOK() {
  return deviceOK;
}

int Lora::getRssi() {
  return sx.getRssi();
}

String Lora::read() {
  if (sleep) return "";
  if (!deviceOK) return String();

  uint8_t packetLenght = 32;
  uint8_t buffer[packetLenght];
  packetLenght = sx.receivePacket(buffer, packetLenght);
  String s;
  for (uint8_t i = 0; i < packetLenght; i++) {
    char c = (char) buffer[i];
    s += c;
    // Serial.print(c);
    // Serial.write(buffer[i]);
  }
  return s;
}

bool Lora::write(String s) {
  if (!deviceOK) return false;
  if (!s.length()) return true;
  if (sleep) powerUp();

  uint8_t buffer[s.length()];
  s.toCharArray(buffer, 32);
  uint8_t packetLength = sizeof(buffer);
  return sx.sendPacket(buffer, packetLength);
}

void Lora::powerUp() {
  if (!sleep) return;
  sleep = false;
  sx.wakeup();
}

void Lora::powerDown() {
  if (sleep) return;
  sleep = true;
  sx.sleep(SX126X_SLEEP_START_WARM); // configuration retention
}

bool Lora::isSleeping() {
  return sleep;
}
