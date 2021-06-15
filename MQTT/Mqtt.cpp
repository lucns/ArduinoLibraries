// Developed by @lucns

#include "Mqtt.h"

Mqtt::Mqtt() {
}

void Mqtt::setCallback(void (*function)(String, String)) {
  pubsubCient.setCallback(function);
}

void Mqtt::connectOnWifi(String ssid, String password) {
  Serial.begin(115200);
  while (!Serial);
  pinMode(BUILTIN_LED, OUTPUT);

  delay(10);
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid.c_str(), password.c_str());

  while (WiFi.status() != WL_CONNECTED) {
    digitalWrite(BUILTIN_LED, HIGH);
    delay(100);
    digitalWrite(BUILTIN_LED, LOW);
    delay(100);
    Serial.print(".");
  }
  Serial.println(".");
  Serial.println("WiFi connected");
}

bool Mqtt::isConnectedOnWifi() {
  return WiFi.status() == WL_CONNECTED;
}

void Mqtt::connectOnBroker(String host, int port, String clientId) {
  pubsubCient.setWifi(wifi);
  pubsubCient.setServer(host.c_str(), port);

  while (!pubsubCient.connected()) {
    Serial.println("Attempting MQTT connection...");
    if (pubsubCient.connect(clientId.c_str())) {
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(pubsubCient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

bool Mqtt::isConnectedOnBroker() {
  return pubsubCient.connected();
}

bool Mqtt::subscribe(String topic) {
  Serial.print("Subscribing ");
  Serial.println(topic);
  return pubsubCient.subscribe(topic.c_str());
}

bool Mqtt::unsubscribe(String topic) {
  return pubsubCient.unsubscribe(topic.c_str());
}

void Mqtt::publish(String topic, String message) {
  pubsubCient.publish(topic.c_str(), message.c_str());
}

void Mqtt::loop() {
  pubsubCient.loop();
}
