#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <SPI.h>

// named constant for the pin the sensor is connected to
const int sensorPin = A0;
// room temperature in Celcius
const float baselineTemp = 30.0;

char ssid[] = "NETGEAR67";     //  your network SSID (name)
char pass[] = "huskywater261";    // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

//#define MQTT_SERVER "192.168.0.8"
byte mac[]    = {  0x90, 0xA2, 0xDA, 0x0E, 0x07, 0x18 };
byte serverRabbit[] = { 192, 168, 0, 4 };//192.168.0.8
byte ip[]     = { 192, 168, 0, 4 };
char* user = "guest";
char* pwd = "guest";

char topicTempHum[] = "arduino-weather-exchange"; //change to a proper name!!!!!!!!!!!!!!!!!!!

// defines and variable for sensor/control mode
#define MODE_OFF    0  // not sensing light, LED off
#define MODE_ON     1  // not sensing light, LED on
#define MODE_SENSE  2  // sensing light, LED controlled by software
int senseMode = 0;
char message_buff[100];
//setup MQTT
WiFiClient wifiClient;
//MQTT
PubSubClient client(serverRabbit, 1883, callback, wifiClient);
  
void setup() {
  // initialize serial and wait for the port to open:
  Serial.begin(9600);
  while (!Serial);
  
  connectToWifi();
  
  // set the LED pins as outputs
  // the for() loop saves some extra coding
  for (int pinNumber = 2; pinNumber < 5; pinNumber++) {
    pinMode(pinNumber, OUTPUT);
    digitalWrite(pinNumber, LOW);
  }
  connectToMQTT();
}

void loop() {
  Serial.println("loop");
  float tempVar = readTemperature();
  char temp[10];
  dtostrf(tempVar, 2, 2, temp);
  Serial.print("DEBUG :: Temp value return is : ");
  Serial.println(temp);
  publishTempHumMsg(temp, "0");
  
  // MQTT client loop processing
  client.loop();
}

void connectToMQTT() {
    
// if (!client.connected()) {
//    delay(1000);
//    Serial.println("Connecting to RabbitMQ");
//    if (client.connect("ArduinoClient", user, pwd)) {
//      Serial.println("Connected to RabbitMQ");
//    } else {
//      Serial.println("Failed to Connect to RabbitMQ");
//    }
// }
if (!client.connected()) {
    delay(1000);
    Serial.println("INFO :: Connecting to RabbitMQ");
    if (client.connect("ArduinoClient", "guest", "guest")) {
      Serial.println("DEBUG :: About to sent message......");
      client.publish("arduino-weather-exchange","Connected to RabbitMQ");
      Serial.println("INFO :: Connected to RabbitMQ");
    } else {
      Serial.println("ERROR :: Failed to Connect to RabbitMQ");
    }
  } else {
    client.publish("outTopic", "hello world");
  }
}

void publishTempHumMsg(String temp, String humidity) {
//    client.subscribe("inTopic");
  //MQTT send json???
  //String pubString = "{\"report\":{\"temp\": \"" + String(temperature) + "\"}}";
  //      pubString.toCharArray(message_buff, pubString.length()+1);
  Serial.println("DEBUG :: Sending temp ");
  Serial.println(temp);
  //"io.m2m/arduino/lightsensor"
  client.publish("arduino-weather-exchange", "temp");
}


void printWifiData() {
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("DEBUG :: IP Address: ");
  Serial.println(ip);
  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("DEBUG :: MAC address: ");
  Serial.print(mac[5], HEX);
  Serial.print(":");
  Serial.print(mac[4], HEX);
  Serial.print(":");
  Serial.print(mac[3], HEX);
  Serial.print(":");
  Serial.print(mac[2], HEX);
  Serial.print(":");
  Serial.print(mac[1], HEX);
  Serial.print(":");
  Serial.println(mac[0], HEX);
}

void printCurrentNet() {
  // print the SSID of the network you're attached to:
  Serial.print("DEBUG :: SSID: ");
  Serial.println(WiFi.SSID());

  // print the MAC address of the router you're attached to:
  byte bssid[6];
  WiFi.BSSID(bssid);
  Serial.print("BSSID: ");
  Serial.print(bssid[5], HEX);
  Serial.print(":");
  Serial.print(bssid[4], HEX);
  Serial.print(":");
  Serial.print(bssid[3], HEX);
  Serial.print(":");
  Serial.print(bssid[2], HEX);
  Serial.print(":");
  Serial.print(bssid[1], HEX);
  Serial.print(":");
  Serial.println(bssid[0], HEX);

  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("DEBUG :: Signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("DEBUG :: Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
}


float readTemperature() {
  // read the value on AnalogIn pin 0
  // and store it in a variable
  int sensorVal = analogRead(sensorPin);

  // send the 10-bit sensor value out the serial port
  Serial.print("DEBUG :: Sensor Value: ");
  Serial.print(sensorVal);

  // convert the ADC reading to voltage
  float voltage = (sensorVal / 1024.0) * 5.0;

  // Send the voltage level out the Serial port
  Serial.print(", Volts: ");
  Serial.print(voltage);

  // convert the voltage to temperature in degrees C
  // the sensor changes 10 mV per degree
  // the datasheet says there's a 500 mV offset
  // ((volatge - 500mV) times 100)
  Serial.print(", degrees C: ");
  float temperature = (voltage - .5) * 100;
  Serial.println(temperature);

  // if the current temperature is lower than the baseline
  // turn off all LEDs
  if (temperature < baselineTemp) {
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
  } // if the temperature rises 2-4 degrees, turn an LED on
  else if (temperature >= baselineTemp + 2 && temperature < baselineTemp + 4) {
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
  } // if the temperature rises 4-6 degrees, turn a second LED on
  else if (temperature >= baselineTemp + 4 && temperature < baselineTemp + 6) {
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(4, LOW);
  } // if the temperature rises more than 6 degrees, turn all LEDs on
  else if (temperature >= baselineTemp + 6) {
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(4, HIGH);
  }
  delay(1000);
  
  //TODO return the temperature value from this function
  return temperature;
}

void connectToWifi() {
  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("Error :: WiFi shield not present");
    while (true);// don't continue:
  }
  // scan for existing networks:
  Serial.println("Scanning available networks...");

  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) {
    Serial.print("INFO :: Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);
    Serial.print("DEBUG :: Status of network connection is ");
    Serial.println(status);
    // wait 10 seconds for connection:
    delay(3000);
  }
  // you're connected now, so print out the data:
  Serial.println("DEBUG :: You're connected to the network");
  printCurrentNet();
  printWifiData();
}

// handles message arrived on subscribed topic(s)
void callback(char* topic, byte* payload, unsigned int length) {

//  int i = 0;
//
//  //Serial.println("Message arrived:  topic: " + String(topic));
//  //Serial.println("Length: " + String(length,DEC));
//
//  // create character buffer with ending null terminator (string)
//  for (i = 0; i < length; i++) {
//    message_buff[i] = payload[i];
//  }
//  message_buff[i] = '\0';
//
//  String msgString = String(message_buff);
//
//  //Serial.println("Payload: " + msgString);
//
//  if (msgString.equals("{\"command\":{\"lightmode\": \"OFF\"}}")) {
//    senseMode = MODE_OFF;
//  } else if (msgString.equals("{\"command\":{\"lightmode\": \"ON\"}}")) {
//    senseMode = MODE_ON;
//  } else if (msgString.equals("{\"command\":{\"lightmode\": \"SENSE\"}}")) {
//    senseMode = MODE_SENSE;
//  }
}

