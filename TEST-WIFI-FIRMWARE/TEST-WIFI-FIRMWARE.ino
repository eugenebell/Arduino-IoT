#include <SPI.h>
#include <WiFi.h>

void setup() {
  //Initialize serial and wait for port to open:
  Serial.begin(9600); 

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present"); 
    // don't continue:
    while(true);
  } 

  Serial.print("Firmware version: ");
  Serial.println(WiFi.firmwareVersion());
}

void loop() {}
