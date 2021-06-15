// Developed by @lucns

#include "Arduino.h"
#include <ESP8266WiFi.h>
#include <PubSubClient.h>

class Mqtt {
  private:
    WiFiClient wifi;
    PubSubClient pubsubCient;
  public:
    Mqtt();
    void setCallback(void (*function)(String, String));
    void connectOnWifi(String ssid, String password);
    bool isConnectedOnWifi();
    void connectOnBroker(String host, int port, String clientId);
    bool isConnectedOnBroker();
    bool subscribe(String topic);
    bool unsubscribe(String topic);
    void publish(String topic, String message);
    void loop();
};
