#include <WiFi.h>

void setup() {
  // Initialize serial communication at 115200 baud
  Serial.begin(115200);
  
  // Wait a moment for the serial monitor to connect
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
