#include "NetworkMqttManager.h"
#include "SensorManager.h"
#include "EEPROM.h"
#include "SystemConfig.h"
#include "EasyNextionLibrary.h"
#include "i2cLogger.h"

bool configTimeFlag = false;
uint32_t lastMQTTConnectionTime = 0;
uint32_t lcdUpdateMillis = 0;
uint32_t sendingIntervalMillis = 0;

// Datalogger needs
I2CLogger logger(0x42);
unsigned long epochTime;


void setup() {

  Serial.begin(115200);
  vTaskDelay(pdMS_TO_TICKS(1000));



  // Initialize Display
  // display.begin(115200);

  pinMode(TIMER_DONE, OUTPUT);
  digitalWrite(TIMER_DONE, LOW);

  // pinMode(LED_STAT_PIN, OUTPUT);
  // digitalWrite(LED_STAT_PIN, LED_OFF);

  pinMode(NETWORK_LED, OUTPUT);
  digitalWrite(NETWORK_LED, LOW);

  

  

  // Init datalogger
  Wire.begin();
  //SD Card Checking
  if (logger.createFile("datalog.txt")) {
    Serial.println("File creation succeeded.");

    //one time rtc set
    //logger.setRTC(1739801500);
  } else {
    Serial.println("File creation failed.");
  }

  vTaskDelay(pdMS_TO_TICKS(500));

  NetworkMode mode = WIFI_GSM;


  // Start the network and MQTT manager task with the specified pins
  Serial.println("🚀 Initializing Network...");
  startNetworkMqttManagerTask(GSM_PWR, NETWORK_LED, mode);

  vTaskDelay(pdMS_TO_TICKS(500));

  // Start the Sensor Manager Task
  startSensorManagerTask();

  vTaskDelay(pdMS_TO_TICKS(500));



}

void loop() {

  // Check Server Connectivity -> Restart when disconnected for too long
  if (networkInfo.mqttGSMConnected || networkInfo.mqttWiFiConnected) {
    lastMQTTConnectionTime = millis();
    digitalWrite(NETWORK_LED, HIGH);
  } else {
    
    digitalWrite(NETWORK_LED, LOW);

    // Check if the ESP has been disconnected for too long
    if (millis() - lastMQTTConnectionTime > MQTT_DISCONNECT_TIMEOUT_MS) {
#ifdef DEBUG_MQTT
      Serial.println("MQTT disconnected for too long. Restarting ESP...");
#endif
      vTaskDelay(100);
      ESP.restart();  // Restart the ESP
    }
  }


  // Update display
  if (millis() - lcdUpdateMillis > 1000) {

    //Update sensorr Values
    display.writeStr("t3.txt", String(sensorData.soil_temperature));
    vTaskDelay(10);
    display.writeStr("t3.txt", String(sensorData.soil_temperature));
    vTaskDelay(10);
    display.writeStr("t4.txt", String(sensorData.soil_ph));
    vTaskDelay(10);
    display.writeStr("t5.txt", String(sensorData.soil_moisture));
    vTaskDelay(10);
    display.writeStr("t8.txt", String(sensorData.soil_nitrogen));
    vTaskDelay(10);
    display.writeStr("t7.txt", String(sensorData.soil_phosphorus));
    vTaskDelay(10);
    display.writeStr("t6.txt", String(sensorData.soil_potassium));
    vTaskDelay(10);
    display.writeStr("t1.txt", String(sensorData.temperature));
    vTaskDelay(10);
    display.writeStr("t2.txt", String(sensorData.humidity));
    vTaskDelay(10);
    


    
    // display.writeNum("p3.pic", GPRS_OK);
    // display.writeNum("p2.pic", WIFI_OK);
    // display.writeNum("p1.pic", MQTT_OK);
    
    //Serial.println("MQTT Connection: " + String(networkInfo.mqttConnected));
    lcdUpdateMillis = millis();
  }


  // Send data to MQTT
  if (sensorDataReady) {
    String payload = "";

    // Format MQTT Payload
    if (xSemaphoreTake(xSemaphore_Sensor, portMAX_DELAY) == pdTRUE) {

      payload = "{\"N\":\"" + String(sensorData.soil_nitrogen)
                + "\", \"P\":\"" + String(sensorData.soil_phosphorus)
                + "\", \"K\":\"" + String(sensorData.soil_potassium)
                + "\", \"SM\":\"" + String(sensorData.soil_moisture)
                + "\", \"ST\":\"" + String(sensorData.soil_temperature)
                + "\", \"T\":\"" + String(sensorData.temperature)
                + "\", \"H\":\"" + String(sensorData.humidity)
                + "\", \"PH\":\"" + String(sensorData.soil_ph)
                + "\"}";

      xSemaphoreGive(xSemaphore_Sensor);
    }


    // Send data to MQTT Server
    if (networkInfo.mqttGSMConnected || networkInfo.mqttWiFiConnected) {
      Serial.println("Sending Data to MQTT");
      Serial.println("Payload: " + payload);

      if(networkInfo.mqttGSMConnected){
        mqttGSM.publish(topicSensorData, payload.c_str());
        Serial.println("Sent MQTT heartbeat message over GSM");
      }

      if(networkInfo.mqttWiFiConnected){
        mqttWiFi.publish(topicSensorData, payload.c_str());
        Serial.println("Sent MQTT heartbeat message over WiFi");
      }

      if (xSemaphoreTake(xSemaphore_Sensor, portMAX_DELAY) == pdTRUE) {

        logger.selectFile("datalog.txt");
        vTaskDelay(1000);

        // Save to SD Card
        String dataToSave = "";
        String rtcTime = String(logger.getRTC());
        dataToSave = rtcTime + ",";
        logger.appendLogData(dataToSave.c_str());
        vTaskDelay(1000);

        dataToSave = String(sensorData.soil_nitrogen) + ",";
        dataToSave += String(sensorData.soil_phosphorus) + ",";
        dataToSave += String(sensorData.soil_potassium) + ",";
        logger.appendLogData(dataToSave.c_str());
        vTaskDelay(1000);

        dataToSave = String(sensorData.soil_moisture, 1) + ",";
        dataToSave += String(sensorData.soil_temperature, 1) + ",";
        dataToSave += String(sensorData.soil_ph, 1) + ",";

        logger.appendLogData(dataToSave.c_str());
        vTaskDelay(1000);


        dataToSave = String(sensorData.temperature, 1) + ",";
        dataToSave += String(sensorData.humidity, 1) + "\n";
        logger.appendLogData(dataToSave.c_str());



        xSemaphoreGive(xSemaphore_Sensor);
      }

      sendingIntervalMillis = millis();

#ifdef DEBUG_SYSTEM
      Serial.println("Shutting down now");
      vTaskDelay(100);  // some time to print
#endif
      //ESP.restart();

      digitalWrite(TIMER_DONE, HIGH); 
    }
  }

  vTaskDelay(pdMS_TO_TICKS(100));
}
