#include <DHT.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <PubSubClient.h>
#include <SPI.h>
#include <MFRC522.h>

#define RST_PIN         9           // Configurable, see typical pin layout above
#define SS_PIN          8          // Configurable, see typical pin layout above

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
byte serverRabbit[] = { 192, 168, 0, 2 };//192.168.0.8
byte ip[]     = { 192, 168, 0, 4 };
char clientName[] = "clientA";
char user[] = "mqtt";
char pwd[] = "mqtt";
int port = 1883;
int tiltState;
//int previousTiltState = 2;

//Time on arduino http://playground.arduino.cc/Code/DateTime

char topicTempHum[] = "arduino-weather"; //change to a proper name!!!!!!!!!!!!!!!!!!!

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
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.
// Number of known default keys (hard-coded)
// NOTE: Synchronize the NR_KNOWN_KEYS define with the defaultKeys[] array
#define NR_KNOWN_KEYS   8
// Known keys, see: https://code.google.com/p/mfcuk/wiki/MifareClassicDefaultKeys
byte knownKeys[NR_KNOWN_KEYS][MFRC522::MF_KEY_SIZE] =  {
    {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, // FF FF FF FF FF FF = factory default
    {0xa0, 0xa1, 0xa2, 0xa3, 0xa4, 0xa5}, // A0 A1 A2 A3 A4 A5
    {0xb0, 0xb1, 0xb2, 0xb3, 0xb4, 0xb5}, // B0 B1 B2 B3 B4 B5
    {0x4d, 0x3a, 0x99, 0xc3, 0x51, 0xdd}, // 4D 3A 99 C3 51 DD
    {0x1a, 0x98, 0x2c, 0x7e, 0x45, 0x9a}, // 1A 98 2C 7E 45 9A
    {0xd3, 0xf7, 0xd3, 0xf7, 0xd3, 0xf7}, // D3 F7 D3 F7 D3 F7
    {0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff}, // AA BB CC DD EE FF
    {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}  // 00 00 00 00 00 00
};
  
void setup() {
  // initialize serial and wait for the port to open:
  Serial.begin(9600);
  while (!Serial);
    
  // set the LED pins as outputs
  // the for() loop saves some extra coding
  for (int pinNumber = 2; pinNumber < 5; pinNumber++) {
    pinMode(pinNumber, OUTPUT);
    digitalWrite(pinNumber, LOW);
  }
  
  connectToWifi();
  connectToMQTT();
  
  setupDHT();
  SPI.begin();                // Init SPI bus
   mfrc522.PCD_Init();         // Init MFRC522 card 
//  setupRFID();

}

void setupDHT() {
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
  checkTiltState();
  String rfid = getRfidUID();
  if (rfid != "") {
    Serial.print("DEBUG :: RFID value return is : ");
    Serial.println(rfid);
    publishRFIDMsg(rfid);
  }
  client.subscribe("arduino-rfid-validation");
  // MQTT client loop processing
  client.loop();
}

String getRfidUID() {
   // Look for new cards
  if ( ! mfrc522.PICC_IsNewCardPresent()) {
     Serial.println("PICC_IsNewCardPresent loop...");
     return "";
  }

    // Select one of the cards
  if ( ! mfrc522.PICC_ReadCardSerial()) {
     Serial.println("PICC_ReadCardSerial loop...");
     return "";
  }
    // Show some details of the PICC (that is: the tag/card)
  
  String rfidUid = "";
  for (byte i = 0; i < mfrc522.uid.size; i++) {
    rfidUid += String(mfrc522.uid.uidByte[i] < 0x10 ? "0" : "");
    rfidUid += String(mfrc522.uid.uidByte[i], HEX);
  }
  Serial.print("Card UID:");
  Serial.println(rfidUid);
  mfrc522.PICC_HaltA();       // Halt PICC
  return rfidUid;
}

void checkTiltState() {
  tiltState = digitalRead(tiltSensorPin);
  Serial.print("INFO :: [1 -> On, 0 -> Off] :: Tilt State is : ");
  Serial.println(tiltState);
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

void connectToMQTT() {
    Serial.println("INFO :: Connecting to RabbitMQ");
    if (client.connect(clientName, user, pwd)) {
      Serial.println("INFO :: Connected to RabbitMQ");
    } else {
      Serial.println("ERROR :: Failed to Connect to RabbitMQ");
    }
}

void publishRFIDMsg(String rfid) {
  //client.subscribe("inTopic");
  Serial.println("DEBUG :: Sending RFID event : ");
  Serial.print(rfid);
  //Format for the message to the following SensorId,temperature,humidity
  String rfidMsg = getSensorId();
  rfidMsg.concat(",");
  rfidMsg.concat(rfid);
//  tempHumMsg.concat(",");
//  tempHumMsg.concat(humidity);
  char message_buff[40];
  rfidMsg.toCharArray(message_buff, rfidMsg.length()+1);
  client.publish("arduino-rfid", message_buff);
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
  client.publish("arduino-weather", message_buff);
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
  //  arduino-rfid-validation
  String resp = String((char*)payload);
  Serial.println("Payload: " + resp);
  Serial.print("---------------BOOM!!!!!!!!!!!!!!!!!!--------------------- : ");
  if (resp == "Valid") {
    digitalWrite(2, HIGH);
  } else {
    digitalWrite(3, HIGH);
  }
  delay(1000);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
}

