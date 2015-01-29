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

//char ssid[] = "NETGEAR67";     //  your network SSID (name)
//char pass[] = "huskywater261";    // your network password
char ssid[] = "AndroidHotspot4559";     //  your network SSID (name)
char pass[] = "64a5676f2d4c";    // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status

//#define MQTT_SERVER "192.168.0.8"
byte mac[]    = {  0x90, 0xA2, 0xDA, 0x0E, 0x07, 0x18 };
//byte serverRabbit[] = { 192, 168, 0, 2 };//192.168.0.8
byte serverRabbit[] = { 104, 131, 80, 167 };
//byte serverRabbit[] = { 192, 168, 43, 246 };
//byte ip[]     = { 192, 168, 0, 4 };
char clientName[] = "clientA";
char user[] = "mqtt";
char pwd[] = "mqtt";
int port = 1883;
int countTempInterval = 0;
int previousTiltState = 2;

//Time on arduino http://playground.arduino.cc/Code/DateTime

char topicTempHum[] = "arduino-weather"; //change to a proper name!!!!!!!!!!!!!!!!!!!

// defines and variable for sensor/control mode
#define MODE_OFF    0  // not sensing light, LED off
#define MODE_ON     1  // not sensing light, LED on
#define MODE_SENSE  2  // sensing light, LED controlled by software
//setup MQTT
WiFiClient wifiClient;
//MQTT
PubSubClient client(serverRabbit, port, callbackFunction, wifiClient);
MFRC522 mfrc522(SS_PIN, RST_PIN);   // Create MFRC522 instance.
  
void setup() {
  // initialize serial and wait for the port to open:
  Serial.begin(9600);
  while (!Serial);
//    listWifiNetworks();
  // set the LED pins as outputs
  // the for() loop saves some extra coding
  for (int pinNumber = 2; pinNumber < 6; pinNumber++) {
    if (pinNumber != 4) {
      pinMode(pinNumber, OUTPUT);
      digitalWrite(pinNumber, LOW);
    }
  }
  connectToWifi();
  connectToMQTT();
  setupDHT();
  SPI.begin();                // Init SPI bus
  mfrc522.PCD_Init();         // Init MFRC522 card 
  ShowReaderDetails();
// client.subscribe("arduino-rfid-validation");

}

void setupDHT() {
  dht.begin();
  // Reading temperature or humidity takes about 250 milliseconds!
  // Sensor readings may also be up to 2 seconds 'old' (its a very slow sensor)
  delay(2000);
}

void loop() {
//  Serial.println("DEBUG :: loop()");
  if (countTempInterval == 5) {
    countTempInterval = 0;
    float tempVar = getTemperature();//readTemperature();
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
  } else {
     countTempInterval += 1; 
//     Serial.print("------------ DEBUG :: countTempInterval is : ");
//     Serial.println(countTempInterval);
  }
  checkTiltState();
  String rfid = getRfidUID();
  if (rfid != "") {
    Serial.print("DEBUG :: RFID value return is : ");
    Serial.println(rfid);
    publishRFIDMsg(rfid);
  } 
//  else {
//    Serial.println("DEBUG :: RFID value return is EMPTY ");
//  }
  //  client.subscribe("arduino-rfid-validation"
  delay(1000);
  //client.loop();
}

String getRfidUID() {
    String rfidUid = "";
   // Look for new cards
  if ( mfrc522.PICC_IsNewCardPresent()) {
     Serial.println("PICC_IsNewCardPresent loop...");
     if ( mfrc522.PICC_ReadCardSerial()) {
       Serial.println("PICC_ReadCardSerial loop...");
       for (byte i = 0; i < mfrc522.uid.size; i++) {
        rfidUid += String(mfrc522.uid.uidByte[i] < 0x10 ? "0" : "");
        rfidUid += String(mfrc522.uid.uidByte[i], HEX);
      }
      Serial.print("Card UID:");
      Serial.println(rfidUid);
      mfrc522.PICC_HaltA();       // Halt PICC
    }
  }
  return rfidUid;
}

void checkTiltState() {
  int tiltState = digitalRead(tiltSensorPin);
//  Serial.print("INFO :: [1 -> On, 0 -> Off] :: Tilt State is : ");
//  Serial.println(tiltState);
  if (previousTiltState != tiltState) {
    previousTiltState = tiltState;
    String tiltMsg = getSensorId();
    tiltMsg.concat(",");
    tiltMsg.concat(tiltState);
    char message_buff[tiltMsg.length()+1];
    tiltMsg.toCharArray(message_buff, tiltMsg.length()+1);
    //send tilt state to the tilt queue
    boolean result = client.publish("arduino-tilt-exchange", message_buff);
    Serial.print("DEBUG :: arduino-tilt-exchange publish outcome : ");
    Serial.println(result);
    if (!result) {
      Serial.print("ERROR :: Retry : ");
      delay(1000);
      result = client.publish("arduino-tilt", message_buff);
      Serial.print("DEBUG :: arduino-tilt publish outcome 2 : ");
      Serial.println(result);
      while ( status != WL_CONNECTED) {
        connectToWifi();
      }
        if (!client.connected()) {
    Serial.print("INFO :: We lost mqtt connection, reconnecting.... ");
    connectToMQTT(); 
  }
    }
  }
}

void connectToMQTT() {
    Serial.println("INFO :: Connecting to RabbitMQ");
    if (client.connect(clientName, user, pwd)) {
      Serial.println("INFO :: Connected to RabbitMQ");
    } else {
      Serial.println("ERROR :: Failed to Connect to RabbitMQ");
      connectToMQTT();
    }
//    client.subscribe("arduino-rfid-validation");
}

void publishRFIDMsg(String rfid) {
  //client.subscribe("inTopic");
  Serial.print("DEBUG :: Sending RFID event : ");
  Serial.println(rfid);
  //Format for the message to the following SensorId,temperature,humidity
  String rfidMsg = getSensorId();
  rfidMsg.concat(",");
  rfidMsg.concat(rfid);
  char message_buff[rfidMsg.length()+1];
  rfidMsg.toCharArray(message_buff, rfidMsg.length()+1);
//  if (!client.connected()) {
//     Serial.print("INFO :: We lost mqtt connection, reconnecting.... ");
//     connectToMQTT(); 
//  }
  boolean result = client.publish("arduino-rfid", message_buff);
   Serial.print("DEBUG :: arduino-rfid publish outcome : ");
   Serial.println(result);
   if (!result) {
    Serial.print("ERROR :: Retry : ");
    delay(1000);
    result = client.publish("arduino-rfid", message_buff);
    Serial.print("DEBUG :: arduino-rfid publish outcome 2 : ");
    Serial.println(result);
    while ( status != WL_CONNECTED) {
        connectToWifi();
      }
        if (!client.connected()) {
    Serial.print("INFO :: We lost mqtt connection, reconnecting.... ");
    connectToMQTT(); 
  }
  }
}

void publishTempHumMsg(String temp, String humidity) {
  //Format for the message to the following SensorId,temperature,humidity
  String tempHumMsg = getSensorId();
  tempHumMsg.concat(",");
  tempHumMsg.concat(temp);
  tempHumMsg.concat(",");
  tempHumMsg.concat(humidity);
  char message_buff[tempHumMsg.length()+1];
  tempHumMsg.toCharArray(message_buff, tempHumMsg.length()+1);
  boolean result = client.publish("arduino-weather", message_buff);
  Serial.print("DEBUG :: arduino-weather publish outcome : ");
  Serial.println(result);
  while (!result) {
    Serial.print("ERROR :: Retry : ");
    delay(1000);
    result = client.publish("arduino-weather", message_buff);
    Serial.print("DEBUG :: arduino-weather publish outcome 2 : ");
    Serial.println(result);
    while ( status != WL_CONNECTED) {
        connectToWifi();
      }
      if (!client.connected()) {
    Serial.print("INFO :: We lost mqtt connection, reconnecting.... ");
    connectToMQTT(); 
  }
  }
}

String getSensorId() {
  return "need-a-sensor-id";
}


float getDHT11HumidityReadings() {
  float h = dht.readHumidity();
  // Check if any reads failed and exit early (to try again).
  if (isnan(h)){// || isnan(t) || isnan(f)) {
    Serial.println("mmmmh!!! should put a retry count of 3 here !!! Failed to read from DHT sensor! looping!!");
    getDHT11HumidityReadings();
  }
  return h;
}

float getTemperature() {
  return dht.readTemperature();
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
  //printWifiData();
}

// handles message arrived on subscribed topic(s)
void callbackFunction(char* topic, byte* payload, unsigned int length) {
  Serial.println("---------------BOOM!!!!!!!!!!!!!!!!!!--------------------- : ");
  //  arduino-rfid-validation
  String resp = String((char*)payload);
  Serial.println("Payload: " + resp);
  
  if (resp == "Valid") {
    digitalWrite(2, HIGH);
  } else {
    digitalWrite(3, HIGH);
  }
  delay(3000);
  digitalWrite(2, LOW);
  digitalWrite(3, LOW);
}

void ShowReaderDetails() {
	// Get the MFRC522 software version
	byte v = mfrc522.PCD_ReadRegister(mfrc522.VersionReg);
	Serial.print("MFRC522 Software Version: 0x");
	Serial.print(v, HEX);
	if (v == 0x91)
		Serial.print(" = v1.0");
	else if (v == 0x92)
		Serial.print(" = v2.0");
	else
		Serial.print(" (unknown)");
	Serial.println("");
	// When 0x00 or 0xFF is returned, communication probably failed
	if ((v == 0x00) || (v == 0xFF)) {
		Serial.println("WARNING: Communication failure, is the MFRC522 properly connected?");
	}
}

//void listWifiNetworks() {
//  // scan for nearby networks:
//  Serial.println("** Scan Networks **");
//  byte numSsid = WiFi.scanNetworks();
//
//  // print the list of networks seen:
//  Serial.print("number of available networks:");
//  Serial.println(numSsid);
//
//  // print the network number and name for each network found:
//  for (int thisNet = 0; thisNet<numSsid; thisNet++) {
//    Serial.print(thisNet);
//    Serial.print(") ");
//    Serial.print(WiFi.SSID(thisNet));
//    Serial.print("\tSignal: ");
//    Serial.print(WiFi.RSSI(thisNet));
//    Serial.print(" dBm");
//    Serial.print("\tEncryption: ");
//    Serial.println(WiFi.encryptionType(thisNet));
//  }
//}


//void printWifiData() {
//  // print your WiFi shield's IP address:
//  IPAddress ip = WiFi.localIP();
//  Serial.print("DEBUG :: IP Address: ");
//  Serial.println(ip);
//  // print MAC address:
//  byte mac[6];
//  WiFi.macAddress(mac);
//  Serial.print("DEBUG :: MAC address: ");
//  Serial.print(mac[5], HEX);
//  Serial.print(":");
//  Serial.print(mac[4], HEX);
//  Serial.print(":");
//  Serial.print(mac[3], HEX);
//  Serial.print(":");
//  Serial.print(mac[2], HEX);
//  Serial.print(":");
//  Serial.print(mac[1], HEX);
//  Serial.print(":");
//  Serial.println(mac[0], HEX);
//    // print the SSID of the network you're attached to:
//  Serial.print("DEBUG :: SSID: ");
//  Serial.println(WiFi.SSID());
//  // print the received signal strength:
//  long rssi = WiFi.RSSI();
//  Serial.print("DEBUG :: Signal strength (RSSI):");
//  Serial.println(rssi);
//  // print the encryption type:
//  byte encryption = WiFi.encryptionType();
//  Serial.print("DEBUG :: Encryption Type:");
//  Serial.println(encryption, HEX);
//}
