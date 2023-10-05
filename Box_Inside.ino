#include <WiFiManager.h>
#include <Ticker.h>
#include <SD.h>
#include "FS.h"
#include <RTClib.h>
#include <Wire.h>
#include <TimeLib.h>  // Add the Time library to work with timestamps
#include <SPI.h>      // Include the SPI library required by the SD library
#include <PubSubClient.h>
#include <LiquidCrystal_I2C.h>
#include "Adafruit_seesaw.h"
#include <Adafruit_AHTX0.h>
#include "DFRobot_ESP_EC.h"           //https://github.com/GreenPonik/DFRobot_ESP_EC_BY_GREENPONIK.git
#include "DFRobot_ESP_PH_WITH_ADC.h"  //https://github.com/GreenPonik/DFRobot_ESP_PH_WITH_ADC_BY_GREENPONIK.git
#include "Adafruit_ADS1X15.h"         //https://github.com/GreenPonik/Adafruit_ADS1X15.git
#include "Wire.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include "EEPROM.h"
#include <ArduinoJson.h>

#define ONE_WIRE_BUS 15
#define LED 2
#define SW1 34
#define address 0x23  //I2C Address 0x23
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

DFRobot_ESP_PH_WITH_ADC ph;
DFRobot_ESP_EC ec;
Adafruit_ADS1115 ads;
Adafruit_ADS1115 ads2;
Ticker ticker;
RTC_DS3231 rtc;


unsigned long intervals[] = {
  1000U,     //0
  2000U,     //1
  3000U,     //2
  5000U,     //3
  10000U,    //4
  15000U,    //5
  20000U,    //6
  25000U,    //7
  60000U,    //8
  1800000U,  //9
};           //this defines the interval for each task in milliseconds
unsigned long last[] = { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
bool calibrationIsRunning = false;


float lastTemperature;

const int relayPin = 33;
const int relay1Pin = 32;
const int relay2Pin = 26;
bool wifiConnected = false;
bool relayState = false;
bool ecButtonPressed = false;  // ตัวแปรเก็บสถานะการกดปุ่ม EC
bool phButtonPressed = false;
bool wifiButtonPressed = false;          // ตัวแปรเก็บสถานะการกดปุ่ม PH
const unsigned long debounceDelay = 50;  // Adjust this as necessary
volatile bool resetWifiFlag = false;
const int chipSelect = 5;  // Choose an appropriate pin number for your hardware setup
const unsigned long restartInterval = 3600000; // 1 hour in milliseconds
unsigned long previousRestartTime = 0;

 
unsigned long lastInterruptTime = 0;
int i = 0;
bool readSerial(char result[]) {
  while (Serial.available() > 0) {
    char inChar = Serial.read();
    if (inChar == '\n') {
      result[i] = '\0';
      Serial.flush();
      i = 0;
      return true;
    }
    if (inChar != '\r') {
      result[i] = inChar;
      i++;
    }
    delay(1);
  }
  return false;
}


LiquidCrystal_I2C lcd(0x27, 20, 4);
Adafruit_seesaw ss;

Adafruit_AHTX0 aht;
sensors_event_t humidity, temp;
uint16_t capread;
char stringcapread[6];
float capreadPercent;
uint8_t buf[4] = { 0 };
uint16_t data, data1;
float Lux;
float tempC;

// MQTT Broker
const char *mqtt_broker = "174.138.17.96";
const char *topic = "relay/control";
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
const int mqtt_port = 1883;
const char *topic_tem2 = "tem2";
const char *topic_capa2 = "capa2";
const char *topic_light2 = "light2";
const char *topic_temp2 = "temp2";
const char *topic_humid2 = "humid2";
const char *topic_ph2 = "ph2";
const char *topic_soilhumid = "soilhumid2";
const char *topic_soiltemp2 = "soiltemp2";
const char *topic_sensor_data2 = "sensor2";

WiFiClient espClient;
PubSubClient client(espClient);
WiFiManager wm;

char result_tem[10];
char result_temp[10];
int light_data;
char result_humid[10];
char result_ph[10];
char result_ec[10];


const int buttonPin = 35;
const int buttonPinWifi = 4;
const int countdownDuration = 15;
const int countdownDuration2 = 90;
Ticker countdownTicker;
int remainingSeconds = countdownDuration;
int remainingSeconds2 = countdownDuration2;



void tick() {
  digitalWrite(LED, !digitalRead(LED));
}

void configModeCallback(WiFiManager *myWiFiManager) {
  Serial.println("Entered config mode");
  Serial.println(WiFi.softAPIP());
  Serial.println(myWiFiManager->getConfigPortalSSID());
  ticker.attach(0.2, tick);
}

void Wifi_Reset_begin() {
  Serial.println("Wifi Reset? Pls. waiting 3S..");
  lcd.clear();
  lcd.print("WiFi Reset in 3 Sec");
  lcd.setCursor(0, 1);
  delay(3000);
  Serial.println("WiFi Reset Setting OK ");
  lcd.clear();
  lcd.print("WiFi Reset Success");
  lcd.setCursor(0, 1);
  WiFiManager wm;
  wm.resetSettings();
  ESP.restart();
}
uint8_t readReg(uint8_t reg, const void *pBuf, size_t size) {
  if (pBuf == NULL) {
    Serial.println("pBuf ERROR!! : null pointer");
  }
  uint8_t *_pBuf = (uint8_t *)pBuf;
  Wire.beginTransmission(address);
  Wire.write(&reg, 1);
  if (Wire.endTransmission() != 0) {
    return 0;
  }
  delay(20);
  Wire.requestFrom(address, (uint8_t)size);
  for (uint16_t i = 0; i < size; i++) {
    _pBuf[i] = Wire.read();
  }
  return size;
}
void writeLogData(fs::FS &fs, const char *path, const char *data) {
  if (!SD.begin(5)) {
    Serial.println("Card Mount Failed");
    return;
  }
}
uint8_t cardType = SD.cardType();
bool shouldSaveConfig = false;

void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

void setup() {
  Serial.begin(4800);
  lcd.begin();
  lcd.backlight();
  Wire.begin();
  rtc.begin();
  rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  aht.begin();
  Serial.println("seesaw Soil Sensor example!");
  ss.begin(0x36);
  Serial.println("ss!");
  EEPROM.begin(64);
  Serial.println("ee");
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);
  pinMode(relay1Pin, OUTPUT);
  digitalWrite(relay1Pin, LOW);
  pinMode(relay2Pin, OUTPUT);
  digitalWrite(relay2Pin, HIGH);
  pinMode(SW1, INPUT_PULLUP);
  ticker.attach(1, tick);
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(buttonPinWifi, INPUT_PULLUP);
  Serial.println("Wifi");
  WiFiManager wm;
  wm.setConnectTimeout(15);
  wm.setConfigPortalTimeout(90);
  wm.stopWebPortal();
  wm.setAPCallback(configModeCallback);
  lcd.clear();
  lcd.print("Please");
  lcd.setCursor(0, 1);
  lcd.print("Connect WiFi");
  // Set software serial baud to 115200;

  unsigned long wifiTimeout = millis() + 300000;
  while (!wifiConnected && millis() < wifiTimeout) {  // 5 minutes timeout
    if (wm.autoConnect("SMART FARM")) {
      wifiConnected = true;
    } else {
      wifiConnected = false;
      for (int i = 0; i < 5; i++) {
        aht.getEvent(&humidity, &temp);  // วัดค่าอุณหภูมิและความชื้น
        Serial.print(temp.temperature);
        Serial.print("                ");
        Serial.println(humidity.relative_humidity);
        uint8_t buf[4] = { 0 };
        uint16_t data, data1;
        float Lux;
        readReg(0x10, buf, 2);  //Register Address 0x10
        data = buf[0] << 8 | buf[1];
        Lux = (((float)data) / 1.2);
        Serial.print("LUX:");
        Serial.print(Lux);
        Serial.print("lx");
        Serial.print("\n");
        delay(500);
        if (!ss.begin(0x36)) {
          tempC = 25;
          Serial.println(F("soil temp sensor not find"));
        }
        float tempC = ss.getTemp();
        uint16_t capread = ss.touchRead(0);
        char stringcapread[6];
        /*Serial.println("*C");
    Serial.print("Capacitive: ");
    Serial.println(capread);
  itoa(capread, stringcapread, 10);*/

        Serial.print("Temperature: ");
        Serial.print(tempC);
        //Serial.print("Temperature: ");
        //Serial.print(tempC);
        //delay(100);
        aht.getEvent(&humidity, &temp);  // วัดค่าอุณหภูมิและความชื้น
        /*Serial.print(temp.temperature);
  Serial.print("                ");
  Serial.println(humidity.relative_humidity);*/

        //delay(1000);

        DateTime rtcNow = rtc.now();
        char timestampBuffer[20];
        sprintf(timestampBuffer, "%04d-%02d-%02d %02d:%02d:%02d",
                rtcNow.year(), rtcNow.month(), rtcNow.day(), rtcNow.hour(), rtcNow.minute(), rtcNow.second());
        String timestamp(timestampBuffer);
        String logData = timestamp + "," + String(temp.temperature) + "," + String(humidity.relative_humidity) + "," + String(Lux);
        writeLogData(SD, "/datalog.txt", logData.c_str());


        // Update logData function to accept individual values for data loggin
        String path = "/datalog.txt";
        if (cardType == CARD_NONE) {
          Serial.println("No SD card attached");
        }

        Serial.print("SD Card Type: ");
        if (cardType == CARD_MMC) {
          Serial.println("MMC");
        } else if (cardType == CARD_SD) {
          Serial.println("SDSC");
        } else if (cardType == CARD_SDHC) {
          Serial.println("SDHC");
        } else {
          Serial.println("UNKNOWN");
        }

        uint64_t cardSize = SD.cardSize() / (1024 * 1024);
        Serial.printf("SD Card Size: %lluMB\n", cardSize);

        Serial.println("SD card initialized successfully!");
        Serial.printf("Writing log data to file: %s\n", path);

        lcd.clear();
        lcd.setCursor(0, 1);
        lcd.print("Temp:");
        lcd.print(temp.temperature);

        lcd.setCursor(0, 2);
        lcd.print("Humid:");
        lcd.print(humidity.relative_humidity);

        lcd.setCursor(0, 0);
        lcd.print("Lux:");
        lcd.print(Lux);
        lcd.setCursor(0, 3);
        lcd.print("Connection lost");
        delay(50);
      }
      ESP.restart();
    }
    if (!wifiConnected) {
      Serial.println("Failed to connect to WiFi within 5 minutes. Proceeding without WiFi.");
    } else {
      // If WiFi connection was successful, detach the ticker.
      ticker.detach();
      Serial.println("");
      Serial.println("WiFi connected.");
      Serial.print("IP address: ");
      Serial.println(WiFi.localIP());
      lcd.clear();
      lcd.print("WiFi Connected");
      lcd.setCursor(0, 1);
    }
    // Connecting to the MQTT broker only if WiFi is connected.
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(callback);
    String client_id = "esp32-client-";
    client_id += String(WiFi.macAddress());
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
      Serial.println("Public emqx mqtt broker connected");
      digitalWrite(relay1Pin, HIGH);
      digitalWrite(relay2Pin, LOW);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Connecting success");
      lcd.clear();
      client.subscribe("relay/control");
    } else {
      digitalWrite(relay2Pin, HIGH);
      digitalWrite(relay1Pin, LOW);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Failed connecting to MQTT");
      Serial.print("Failed with state ");
      Serial.print(client.state());
      lcd.clear();
    }
  }
}

// publish and subscribe
//client.publish(topic, "100");

void callback(char *topic, byte *payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  if (message == "ON") {
    digitalWrite(relayPin, LOW);
    Serial.println("รดน้ำ");
    relayState = true;
    client.publish("relay/status2", "รดน้ำ");
  } else if (message == "OFF") {
    digitalWrite(relayPin, HIGH);
    Serial.println("หยุดรดน้ำ");
    relayState = false;
    client.publish("relay/status2", "หยุดรดน้ำ");
  }
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  Serial.println("-----------------------");
}





void Soil() {
  if (!ss.begin(0x36)) {
    tempC = 25;
    //Serial.println(F("soil temp sensor not find"));
    return;
  }
  
  tempC = ss.getTemp();
  capread = ss.touchRead(0);

  Serial.println("*C");
  Serial.print("Capacitive: ");
  Serial.println(capread);

  // Convert capread to percentage
  capreadPercent = ((capread - 200.0) / (2000.0 - 200.0)) * 100.0;

  Serial.print("Temperature: ");
  Serial.print(tempC);

  // Format tempC with two decimal places using dtostrf
  result_tem[10];
  dtostrf(tempC, 6, 2, result_tem);

  delay(100);

  Serial.print("Capacitive (%): ");
  Serial.print(capreadPercent);  // Display capreadPercent with two decimal places
  //delay(100);
}


void Light() {
  readReg(0x10, buf, 2);  //Register Address 0x10
  data = buf[0] << 8 | buf[1];
  Lux = (((float)data) / 1.2);
  /*Serial.print("LUX:");
    Serial.print(Lux);
    Serial.print("lx");
    Serial.print("\n");*/
  char msg[50];
  snprintf(msg, 50, "%.2f lx", Lux);
  client.publish(topic_light2, msg);
}

void Tempera() {
  aht.getEvent(&humidity, &temp);  // วัดค่าอุณหภูมิและความชื้น
  /*Serial.print(temp.temperature);
  Serial.print("                ");
  Serial.println(humidity.relative_humidity);*/

  dtostrf(temp.temperature, 6, 2, result_temp);
  client.publish(topic_temp2, result_temp);

  dtostrf(humidity.relative_humidity, 6, 2, result_humid);
  client.publish(topic_humid2, result_humid);
  //delay(1000);
}



String getCurrentTimestamp() {
  DateTime now = rtc.now();
  char timestampBuffer[20];
  sprintf(timestampBuffer, "%04d-%02d-%02d %02d:%02d:%02d",
          now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  return String(timestampBuffer);
}


// Update logData function to accept individual values for data logging
void display() {

  lcd.clear();
  lcd.setCursor(0, 1);
  lcd.print("Temp:");
  lcd.print(temp.temperature);

  lcd.setCursor(0, 2);
  lcd.print("Humid:");
  lcd.print(humidity.relative_humidity);

  lcd.setCursor(0, 0);
  lcd.print("Lux:");
  lcd.print(Lux);

  lcd.setCursor(0, 3);
  lcd.print("Stemp:");
  lcd.print(result_tem);

  lcd.setCursor(12, 3);
  lcd.print("SHumid:");
  lcd.print(capreadPercent);
}

void publishSensorData(const char *topic, const JsonObject &data) {
  char jsonBuffer[256];
  size_t len = measureJson(data);

  // Use a DynamicJsonDocument to temporarily hold the serialized data
  DynamicJsonDocument tempDoc(256);
  tempDoc.set(data);  // Copy the data from the provided JsonObject

  // Serialize the JSON data into the buffer
  serializeJson(tempDoc, jsonBuffer, sizeof(jsonBuffer));

  client.publish(topic, jsonBuffer);  // Publish the JSON data
}




void loop() {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  const unsigned long interval = 5 * 60 * 1000;
  const unsigned long interval2 = 3 * 1000;
    if (currentMillis - previousRestartTime >= restartInterval) {
    Serial.println("Restarting ESP32...");
    ESP.restart(); // Restart the ESP32
    previousRestartTime = currentMillis;
  }

    if (digitalRead(buttonPin) == HIGH) {
    Serial.println("Reset button pressed. Restarting...");
    delay(1000);  // Add a small delay to debounce the button
    ESP.restart();
  }

  if (digitalRead(buttonPinWifi) == LOW) {
    if (!wifiButtonPressed) {
      wifiButtonPressed = true;
      Wifi_Reset_begin();
    }
  } else {
    wifiButtonPressed = false;
  }
  client.loop();
  if (shouldSaveConfig) {
    // Restart the ESP32
    ESP.restart();
  }  // 20 seconds interval in milliseconds

    unsigned long currentMillis2 = millis();
    if (currentMillis2 - previousMillis >= interval2) {
      previousMillis = currentMillis2;
      Tempera();
      Soil();
      Light();
      display();
    }
    // Assuming you have sensor readings in variables: temperature, humidity, Lux, phValue, ecValue
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi connection lost. Reconnecting...");
        unsigned long currentMillis2 = millis();
        if (currentMillis2 - previousMillis >= interval2) {
          previousMillis = currentMillis2;
          digitalWrite(relay2Pin, HIGH);
          Tempera();
          Soil();
          Light();
          display();
          lcd.setCursor(0, 3);
          lcd.print("Connection lost");
        }
        if (currentMillis - previousMillis >= interval) {
          previousMillis = currentMillis;

          // Assuming you have sensor readings in variables: temperature, humidity, Lux, phValue, ecValue
          DateTime now = rtc.now();
          char timestampBuffer[20];
          sprintf(timestampBuffer, "%04d-%02d-%02d %02d:%02d:%02d",
                  now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
          String timestamp = getCurrentTimestamp();
          String logData = timestamp + "," + String(temp.temperature) + "," + String(humidity.relative_humidity) + "," + String(Lux) + "," + String(tempC) + "," + String(capread);
          writeLogData(SD, "/datalog.txt", logData.c_str());
          WiFi.reconnect();// Attempt to reconnect
        }
    }
  
  DynamicJsonDocument sensorDataJson(256);
  sensorDataJson["Temperature"] = temp.temperature;
  sensorDataJson["Humidity"] = humidity.relative_humidity;
  sensorDataJson["Light"] = Lux;
  sensorDataJson["SoilTemp"] = tempC;
  sensorDataJson["SoilHumid"] = capreadPercent;

  sensorDataJson.remove("InvalidProperty");  // Remove any invalid properties


  // Publish sensor data
  publishSensorData(topic_sensor_data2, sensorDataJson.as<JsonObject>());


  delay(5000);
  Serial.println("loop");  // Adjust delay as needed
}
