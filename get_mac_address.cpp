#include <WiFi.h>

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println();
  Serial.println("=========================================");
  Serial.print("ESP32 Camera MAC Address: ");
  Serial.println(WiFi.macAddress());
  Serial.println("=========================================");
}

void loop() {
  // Nothing to do here
}
