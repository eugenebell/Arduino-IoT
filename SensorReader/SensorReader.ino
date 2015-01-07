#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>
#include <WiFiUdp.h>

#include <PubSubClient.h>


#include <SPI.h>

/*

 */

// named constant for the pin the sensor is connected to
const int sensorPin = A0;
// room temperature in Celcius
const float baselineTemp = 30.0;

char ssid[] = "NETGEAR67";     //  your network SSID (name)
char pass[] = "huskywater261";    // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

//MQTT
PubSubClient client;
//#define MQTT_SERVER "192.168.0.8"
byte mac[]    = {  0xDE, 0xED, 0xBA, 0xFE, 0xFE, 0xED };
byte serverRabbit[] = { 192, 168, 0, 4 };//192.168.0.8
char server[] = "www.google.com";
byte ip[]     = { 192, 168, 0, 4 };
char* user = "guest";
char* pwd = "guest";

char topicTempHum[] = "hello/world"; //change to a proper name!!!!!!!!!!!!!!!!!!!

// defines and variable for sensor/control mode
#define MODE_OFF    0  // not sensing light, LED off
#define MODE_ON     1  // not sensing light, LED on
#define MODE_SENSE  2  // sensing light, LED controlled by software
int senseMode = 0;
char message_buff[100];
//setup MQTT
  WiFiClient wifiClient;
  
void setup() {
  // initialize serial and wait for the port to open:
  Serial.begin(9600);
  while (!Serial) ;

  // check for the presence of the shield:
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue:
    while (true);
  }

  // scan for existing networks:
  Serial.println("Scanning available networks...");

  // attempt to connect to Wifi network:
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network:
    status = WiFi.begin(ssid, pass);

    // wait 10 seconds for connection:
    delay(5000);
  }

  // you're connected now, so print out the data:
  Serial.print("You're connected to the network");
  printCurrentNet();
  printWifiData();

  // set the LED pins as outputs
  // the for() loop saves some extra coding
  for (int pinNumber = 2; pinNumber < 5; pinNumber++) {
    pinMode(pinNumber, OUTPUT);
    digitalWrite(pinNumber, LOW);
  }

  
//  client = PubSubClient(serverRabbit, 1883, callback, wifiClient);
// if (!client.connected()) {
//    delay(1000);
//    Serial.println("Connecting to RabbitMQ");
//    if (client.connect("ArduinoClient", user, pwd)) {
//      Serial.println("Connected to RabbitMQ");
//    } else {
//      Serial.println("Failed to Connect to RabbitMQ");
//    }
// }
 
 if (wifiClient.connect(server, 80)) {
    Serial.println("connected");
    // Make a HTTP request:
    wifiClient.println("GET /search?q=arduino HTTP/1.1");
    wifiClient.println("Host: www.google.com");
    wifiClient.println("Connection: close");
    wifiClient.println();
  } 
  else {
    // kf you didn't get a connection to the server:
    Serial.println("WIFI google connection failed");
  }
}

void loop() {
  Serial.println("loop");
  
  if (wifiClient.available()) {
    char c = wifiClient.read();
    Serial.print(c);
  }
  
  float tempVar = readTemperature();
  char temp[10];
  dtostrf(tempVar, 2, 2, temp);
  Serial.print("Temp value return is : ");
  Serial.println(temp);
//  publishTempHumMsg(temp, "0");
}


void publishTempHumMsg(String temp, String humidity) {
  
  if (!client.connected()) {
    delay(1000);
    Serial.println("Connecting to RabbitMQ");
    if (client.connect("ArduinoClient", user, pwd)) {
      client.publish("topic","Connected to RabbitMQ");
      Serial.println("Connected to RabbitMQ");
    } else {
      Serial.println("Failed to Connect to RabbitMQ");
    }
  } else {
    client.publish("outTopic", "hello world");
  }
  
//  if (client.connect("arduinoClient", "guest", "guest")) {
//    Serial.println("connected to rabbit!!!");
//    client.publish("outTopic", "hello world");
//    client.subscribe("inTopic");
//  } else {
//    Serial.println("Failed to connect!!! 101");
//  }
  //MQTT send json???
  //String pubString = "{\"report\":{\"temp\": \"" + String(temperature) + "\"}}";
  //      pubString.toCharArray(message_buff, pubString.length()+1);
  Serial.println("sending temp ");
  Serial.println(temp);
  //"io.m2m/arduino/lightsensor"
  client.publish("topicTempHum", "temp");

  // MQTT client loop processing
 // client.loop();
}

void printMacAddress() {
  // the MAC address of your Wifi shield
  byte mac[6];

  // print your MAC address:
  WiFi.macAddress(mac);
  Serial.print("MAC: ");
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


void printWifiData() {
  Serial.println("------> printWifiData s");
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
  //Serial.println(ip);

  // print your MAC address:
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print("MAC address: ");
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
  Serial.println("------> printWifiData e");

}

void printCurrentNet() {
  Serial.println("------> printCurrentNet s");
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
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
  Serial.print("signal strength (RSSI):");
  Serial.println(rssi);

  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("Encryption Type:");
  Serial.println(encryption, HEX);
  Serial.println();
  Serial.println("------> printCurrentNet e");
}


float readTemperature() {
  // read the value on AnalogIn pin 0
  // and store it in a variable
  int sensorVal = analogRead(sensorPin);

  // send the 10-bit sensor value out the serial port
  Serial.print("sensor Value: ");
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

