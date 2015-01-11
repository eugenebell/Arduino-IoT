#include <DHT.h>


#include <WiFi.h>
#include <WiFiClient.h>
//#include <WiFiServer.h>
//#include <WiFiUdp.h>
#include <PubSubClient.h>
#include <SPI.h>

#define DHTTYPE DHT11   // DHT 11 
#define DHTPIN A1 
DHT dht(DHTPIN, DHTTYPE);

// named constant for the pin the sensor is connected to
const int sensorPin = A0;
const int tiltSensorPin = A2;
// room temperature in Celcius
const float baselineTemp = 30.0;

char ssid[] = "NETGEAR67";     //  your network SSID (name)
char pass[] = "huskywater261";    // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

//#define MQTT_SERVER "192.168.0.8"
byte mac[]    = {  0x90, 0xA2, 0xDA, 0x0E, 0x07, 0x18 };
byte serverRabbit[] = { 192, 168, 0, 4 };//192.168.0.8
byte ip[]     = { 192, 168, 0, 4 };
//char* user = "guest";
//char* pwd = "guest";
char clientName[] = "clientA";
char user[] = "mqtt";
char pwd[] = "mqtt";
int port = 1883;
int tiltState;
int previousTiltState = 2;

//Time on arduino http://playground.arduino.cc/Code/DateTime

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
PubSubClient client(serverRabbit, port, callback, wifiClient);
  
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
  dht.begin();
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  delay(2000);
}

void loop() {
  Serial.println("DEBUG :: loop()");
  float tempVar = readTemperature();
  float humVar = getDHT11HumidityReadings();
  char temp[10];
  char humidity[10];
  dtostrf(tempVar, 2, 2, temp);
  dtostrf(humVar, 2, 2, humidity);
  Serial.print("DEBUG :: Temp value return is : ");
  Serial.println(temp);
  Serial.print("DEBUG :: Humidity value return is : ");
  Serial.println(humidity);
  publishTempHumMsg(temp, humidity);
  // MQTT client loop processing
  client.loop();
}

void checkTiltState() {
  tiltState = digitalRead(tiltSensorPin);
  Serial.print("INFO :: [1 -> On, 0 -> Off] :: Tilt State is : ");
  Serial.println(tiltState);
  if (tiltState != previousTiltState) {
    previousTiltState = tiltState;
    String tiltMsg = getSensorId();
    tiltMsg.concat(",");
    tiltMsg.concat(tiltState);
//    tiltMsg.concat(",");
//    tiltMsg.concat(time);
    char message_buff[30];
    tiltMsg.toCharArray(message_buff, tiltMsg.length()+1);
    //send tilt state to the tilt queue
    client.publish("arduino-tilt-exchange", message_buff);
  }
  
}

void connectToMQTT() {
    Serial.println("INFO :: Connecting to RabbitMQ");
    if (client.connect(clientName, user, pwd)) {
      Serial.println("INFO :: Connected to RabbitMQ");
    } else {
      Serial.println("ERROR :: Failed to Connect to RabbitMQ");
    }
}

void publishTempHumMsg(String temp, String humidity) {
  //client.subscribe("inTopic");
  Serial.println("DEBUG :: Sending temp ");
  Serial.println(temp);
  //Format for the message to the following SensorId,temperature,humidity
  String tempHumMsg = getSensorId();
  tempHumMsg.concat(",");
  tempHumMsg.concat(temp);
  tempHumMsg.concat(",");
  tempHumMsg.concat(humidity);
  char message_buff[80];
  tempHumMsg.toCharArray(message_buff, tempHumMsg.length()+1);
  client.publish("arduino-weather-exchange", message_buff);
}

String getSensorId() {
  return "need-a-sensor-id";
}


void printWifiData() {
  // print your WiFi shield's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("DEBUG :: IP Address: ");
  Serial.println(ip);
  // print MAC address:
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
    // print the SSID of the network you're attached to:
  Serial.print("DEBUG :: SSID: ");
  Serial.println(WiFi.SSID());
  // print the received signal strength:
  long rssi = WiFi.RSSI();
  Serial.print("DEBUG :: Signal strength (RSSI):");
  Serial.println(rssi);
  // print the encryption type:
  byte encryption = WiFi.encryptionType();
  Serial.print("DEBUG :: Encryption Type:");
  Serial.println(encryption, HEX);
}

float getDHT11HumidityReadings() {
  float h = dht.readHumidity();
  // Read temperature as Celsius
  float t = dht.readTemperature();
  // Read temperature as Fahrenheit
  float f = dht.readTemperature(true);
  
  // Check if any reads failed and exit early (to try again).
  if (isnan(h) || isnan(t) || isnan(f)) {
    Serial.println("mmmmh!!! should put a retry count of 3 here !!! Failed to read from DHT sensor! looping!!");
    getDHT11HumidityReadings();
  }

  // Compute heat index
  // Must send in temp in Fahrenheit!
  float hi = dht.computeHeatIndex(f, h);

  Serial.print("Humidity: "); 
  Serial.print(h);
  Serial.print(" %\t");
  Serial.print("Temperature: "); 
  Serial.print(t);
  Serial.print(" *C ");
  Serial.print(f);
  Serial.print(" *F\t");
  Serial.print("Heat index: ");
  Serial.print(hi);
  Serial.println(" *F");
  delay(500);
 return h;
}

float readTemperature() {
  // read the value on AnalogIn pin 0 and store it in a variable
  int sensorVal = analogRead(sensorPin);
  // send the 10-bit sensor value out the serial port
  Serial.print("DEBUG :: Sensor Value: ");
  Serial.print(sensorVal);
  // convert the ADC reading to voltage
  float voltage = (sensorVal / 1024.0) * 5.0;
  // Send the voltage level out the Serial port
  Serial.print(", Volts: ");
  Serial.print(voltage);

  // convert the voltage to temperature in degrees C the sensor changes 10 mV per degree
  // the datasheet says there's a 500 mV offset ((volatge - 500mV) times 100)
  Serial.print(", degrees C: ");
  float temperature = (voltage - .5) * 100;
  Serial.println(temperature);

  // if the current temperature is lower than the baseline turn off all LEDs
  if (temperature < baselineTemp) { // if the temperature rises 2-4 degrees, turn an LED on
    digitalWrite(2, LOW);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
  } else if (temperature >= baselineTemp + 2 && temperature < baselineTemp + 4) {
    digitalWrite(2, HIGH);
    digitalWrite(3, LOW);
    digitalWrite(4, LOW);
  }  else if (temperature >= baselineTemp + 4 && temperature < baselineTemp + 6) { // if the temperature rises 4-6 degrees, turn a second LED on
    digitalWrite(2, HIGH);
    digitalWrite(3, HIGH);
    digitalWrite(4, LOW);
  } else if (temperature >= baselineTemp + 6) { // if the temperature rises more than 6 degrees, turn all LEDs on
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
    delay(6000);
  }
  // you're connected now, so print out the data:
  Serial.println("DEBUG :: You're connected to the network");
  printWifiData();
}

// handles message arrived on subscribed topic(s)
void callback(char* topic, byte* payload, unsigned int length) {
}

