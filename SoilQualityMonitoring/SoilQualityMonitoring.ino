#include "NetworkManager.h"
#include "SensorManager.h"
#include "MQTTMonitor.h"
#include "EasyNextionLibrary.h"
#include "EEPROM.h"
#include "i2cLogger.h"
#include "SystemConfig.h"

#include <Watchdog.h>

// Variable to save current epoch time
//unsigned long epochTime;

// Initialize watchdog to  seconds
//Watchdog wdt(WDT_TIME_MS);

// Initialize Nextion
EasyNex display(Serial2);  // Hardware Serial 16 -> RX of Nextion
                           // Hardware Serial 17 -> TX of Nextion

I2CLogger logger(0x42);


MQTTHandler mqttHandler;

bool configTimeFlag = false;

// Global variables to store min and max temperature
float minTemperature = 1000.0;   // Initialize with a very high value
float maxTemperature = -1000.0;  // Initialize with a very low value
float minHumidity = 1000.0;
float maxHumidity = -1000.0;

uint32_t sendInterval = 10000;
uint32_t lastMQTTConnectionTime = 0;

void updateMinMaxValues(float currentTemperature, float currentHumidity, unsigned long epochTime);
uint32_t readSendIntervalFromEEPROM();
void writeSendIntervalToEEPROM(uint32_t interval);

unsigned long lastDay = 0;  // Track the last recorded day (epoch)

void setup() {
  Serial.begin(115200);
  delay(1000);

  EEPROM.begin(EEPROM_SIZE);

  Wire.begin();

  //SD Card Checking
  // Test creating a file
  if (logger.createFile("test.txt")) {
    Serial.println("File creation succeeded.");
  } else {
    Serial.println("File creation failed.");
  }
  delay(1000);


  // Read `sendInterval` from EEPROM
  sendInterval = readSendIntervalFromEEPROM();

  if (sendInterval < 10000 || sendInterval >= 3600000) {  // Minimum Sending time is 10s
    sendInterval = 10000;
    EEPROM.put(EEPROM_SEND_INTERVAL_ADDR, sendInterval);
    EEPROM.commit();
  }

  Serial.println("Sending Interval: " + String(sendInterval));

  display.begin(9600);

  // Initialize Watchdog Timer
  //wdt.begin();
  //esp_task_wdt_add(NULL);

  // Start the Network Monitor Task
  startNetworkMonitorTask();

  // Start the Sensor Manager Task
  startSensorManagerTask();

  // Initialize MQTT
  mqttHandler.init(mqttServer, mqttPort, mqttUser, mqttPassword, deviceESN);

  // Set the initial message to publish on connect
  mqttHandler.setInitialMessage("SQM/init", "Device SQM-001 is now connected");

  // // Add subscription topics
  mqttHandler.addSubscriptionTopic("SQM/SQM-001/interval");
  mqttHandler.addSubscriptionTopic("Innov/GiveTime");

  // Start the MQTT Monitor Task
  startMQTTMonitorTask();
}


uint32_t lastSendTime = 0;


void loop() {

  // Check Wifi Connectivity
  if (networkInfo.wifiConnected) {
    display.writeNum("p3.pic", WIFI_OK);
    Serial.println("Wi-Fi is connected");
    Serial.println("SSID: " + networkInfo.SSID);
    Serial.println("RSSI: " + String(networkInfo.RSSI));

  } else {
    display.writeNum("p3.pic", WIFI_NOK);
    Serial.println("Wi-Fi is disconnected");
  }

  // Check Server Connectivity -> Restart when disconnected for too long
  if (networkInfo.wifiConnected && mqttHandler.checkConnectivity()) {

    lastMQTTConnectionTime = millis();

    display.writeNum("p4.pic", SERVER_OK);
    mqttHandler.publish("Innov/GetTime", " ");
  } else {
    display.writeNum("p4.pic", SERVER_NOK);

    // Check if the ESP has been disconnected for too long
    if (millis() - lastMQTTConnectionTime > MQTT_DISCONNECT_TIMEOUT_MS) {
      Serial.println("MQTT disconnected for too long. Restarting ESP...");
      vTaskDelay(100);
      ESP.restart();  // Restart the ESP
    }
  }

  // Update Display
  if (xSemaphoreTake(xSemaphore_Sensor, portMAX_DELAY) == pdTRUE) {

    // Update Sensor on Display
    splitFloat(sensorData.temperature, numWhole, numDecimal);
    display.writeStr("t1.txt", String(numWhole));
    display.writeStr("t2.txt", "." + String(numDecimal));
    splitFloat(sensorData.humidity, numWhole, numDecimal);
    display.writeStr("t4.txt", String(numWhole));
    display.writeStr("t3.txt", "." + String(numDecimal));
    display.writeNum("p1.pic", TEMPERATURE_LABEL);
    display.writeNum("p5.pic", HUMIDITY_LABEL);

    // Update Min and Max Values
    display.writeStr("t5.txt", String(maxTemperature, 1));
    display.writeStr("t6.txt", String(maxHumidity, 1));
    display.writeStr("t8.txt", String(minTemperature, 1));
    display.writeStr("t7.txt", String(minHumidity, 1));
    xSemaphoreGive(xSemaphore_Sensor);
  }

  // Update Local Time
  if (xSemaphoreTake(xSemaphore_LocalTime, portMAX_DELAY) == pdTRUE) {
    display.writeStr("t0.txt", localTime);
    xSemaphoreGive(xSemaphore_LocalTime);
  }

  //Send data to server
  if (millis() - lastSendTime > sendInterval) {
    lastSendTime = millis();
    String topic = String("SQM/") + deviceESN;
    String payload = "";
    if (xSemaphoreTake(xSemaphore_Sensor, portMAX_DELAY) == pdTRUE) {

      payload = "{\"N\":\"" + String(sensorData.soil_nitrogen)
                + "\", \"P\":\"" + String(sensorData.soil_phosphorus)
                + "\", \"K\":\"" + String(sensorData.soil_potassium)
                + "\", \"SM\":\"" + String(sensorData.soil_moisture)
                + "\", \"ST\":\"" + String(sensorData.soil_temperature)
                + "\", \"T\":\"" + String(sensorData.temperature)
                + "\", \"H\":\"" + String(sensorData.humidity)
                + "\", \"PH\":\"" + String(sensorData.soil_ph)
                + "}";

      xSemaphoreGive(xSemaphore_Sensor);
    }
    if (mqttHandler.checkConnectivity()) {
      mqttHandler.publish(topic.c_str(), payload.c_str());
#ifdef DEBUG_MQTT
      Serial.println("Sent heartbeat message.");
#endif
    }
  }

  vTaskDelay(5000);
}


// Function to split float into whole and decimal parts
void splitFloat(float number, int &wholePart, int &decimalPart) {
  wholePart = (int)number;                              // Extract the whole number part
  decimalPart = abs((int)((number - wholePart) * 10));  // Extract the decimal part
}

// Read `sendInterval` from EEPROM
uint32_t readSendIntervalFromEEPROM() {
  uint32_t interval;
  EEPROM.get(EEPROM_SEND_INTERVAL_ADDR, interval);
  if (interval == 0 || interval > 3600000) {  // Set a default if the value is invalid
    interval = 60000;                         // Default to 60 seconds
    EEPROM.put(EEPROM_SEND_INTERVAL_ADDR, interval);
    EEPROM.commit();
  }
  return interval;
}
