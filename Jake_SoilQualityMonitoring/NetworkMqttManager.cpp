#include <WiFi.h>
#include "NetworkMqttManager.h"
#include "SystemConfig.h"

#include "esp_task_wdt.h"

// Include GSM library
#define TINY_GSM_MODEM_SIM800
#include <TinyGsmClient.h>

// Serial ports
#define SerialMon Serial
#define GSMSerial Serial1

// Debugging
#define TINY_GSM_DEBUG SerialMon

// GSM credentials
const char apn[] = "Internet";
const char gprsUser[] = "";
const char gprsPass[] = "";

// Globals
bool isMQTTConnectedWiFi = false;
bool isMQTTConnectedGSM = false;
bool wifiConnected = false;
bool gsmConnected = false;

NetworkInfo_t networkInfo = {false, false, false, false, " ", " ", 0};

// Clients
TinyGsm modem(GSMSerial);
TinyGsmClient gsmClient(modem);
WiFiClient wifiClient;
PubSubClient mqttWiFi(wifiClient);
PubSubClient mqttGSM(gsmClient);

int ledPin = -1;
int powerPin = -1;

void mqttCallback(char* topic, byte* payload, unsigned int len) {
  SerialMon.printf("Message arrived [%s]: ", topic);
  SerialMon.write(payload, len);
  SerialMon.println();
}

bool connectWiFi() {
  SerialMon.println("Attempting WiFi connection in the background...");
  WiFi.begin(wifiSSID, wifiPass);
  return true;  // No need to wait, let it run in the background
}

bool connectGPRS() {
  SerialMon.println("Connecting to GPRS...");
  GSMSerial.begin(9600);

  if (!modem.waitForNetwork(10000)) {  // 10s timeout
    SerialMon.println("Failed to connect to GSM network.");
    return false;
  }

  if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
    SerialMon.println("Failed to connect to GPRS.");
    return false;
  }

  SerialMon.println("Connected to GPRS.");
  return true;
}

bool mqttConnect(PubSubClient& mqttClient, uint8_t connectivity) {
  mqttClient.setKeepAlive(60);

  switch (connectivity) {
    case WIFI_CONNECTED:
      SerialMon.println("Connecting to MQTT broker via WiFi...");
      if (mqttClient.connect("JakeWiFiClient", "mqtt", "ICPHmqtt!")) {
        SerialMon.println("Connected to MQTT broker via WiFi");
        mqttClient.publish(topicInit, "Wi-Fi MQTT Client Started");
        mqttClient.subscribe(topicConfig);
        return true;
      }
      SerialMon.println("Failed to connect to MQTT broker via WiFi");
      break;
    case GSM_CONNECTED:
      SerialMon.println("Connecting to MQTT broker via GSM...");
      if (mqttClient.connect("JakeGSMClient", "mqtt", "ICPHmqtt!")) {
        SerialMon.println("Connected to MQTT broker via GSM");
        mqttClient.publish(topicInit, "GSM MQTT Client Started");
        mqttClient.subscribe(topicConfig);
        return true;
      }
      SerialMon.println("Failed to connect to MQTT broker via GSM");
      break;
    default:
      return false;
  }
  return false;
}

void wifiTask(void* pvParameters) {
  Serial.printf("WiFi Task Running on Core %d\n", xPortGetCoreID());

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();  // Ensure fresh connection attempt
  vTaskDelay(pdMS_TO_TICKS(1000));

  WiFi.begin(wifiSSID, wifiPass);
  Serial.println("Attempting WiFi connection...");


  while (true) {
    if (WiFi.status() == WL_CONNECTED) {
      if (!wifiConnected) {
        Serial.println("WiFi Connected!");
        wifiConnected = true;
        mqttWiFi.setClient(wifiClient);
        mqttWiFi.setServer(mqttServer, mqttPort);
        mqttWiFi.setCallback(mqttCallback);
      }
    } else {
      if (wifiConnected) {
        Serial.println("WiFi Disconnected! Retrying...");
        wifiConnected = false;
        WiFi.disconnect();
        WiFi.begin(wifiSSID, wifiPass);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(2000));  // Retry every 2 seconds
  }
}


void gsmTask(void* pvParameters) {
  if (powerPin != -1) {
    pinMode(powerPin, OUTPUT);
  }



  while (true) {
    Serial.println("📶 Attempting GSM Connection...");

    GSMSerial.begin(9600);
    vTaskDelay(pdMS_TO_TICKS(1000));

    if (modem.testAT()) {
      Serial.println("GSM is ON... Bypass toggling");
    } else {

      // Toggle Power Key (Only if needed)
      for (int attempt = 0; attempt < 3; attempt++) {
        SerialMon.println("Toggling GSM Power Key...");
        digitalWrite(powerPin, LOW);
        vTaskDelay(pdMS_TO_TICKS(2000));
        digitalWrite(powerPin, HIGH);
        vTaskDelay(pdMS_TO_TICKS(2000));
        digitalWrite(powerPin, LOW);
        vTaskDelay(pdMS_TO_TICKS(2000));

        GSMSerial.begin(9600);
        vTaskDelay(pdMS_TO_TICKS(1000));

        if (modem.testAT()) {
          SerialMon.println("GSM modem is ON.");
          break;
        }
      }
    }

    // Keep trying until GSM connects
    if (!connectGPRS()) {
      Serial.println("🚨 GSM Connection Failed! Retrying in 10s...");
      vTaskDelay(pdMS_TO_TICKS(10000));  // Wait 10 seconds before retrying
      continue;                          // Restart the loop
    }

    Serial.println("✅ GSM Connected!");
    gsmConnected = true;
    mqttGSM.setClient(gsmClient);
    mqttGSM.setServer(mqttServer, mqttPort);
    mqttGSM.setCallback(mqttCallback);

    // Keep resetting watchdog
    while (true) {

      vTaskDelay(pdMS_TO_TICKS(5000));
    }
  }
}




void mqttTask(void* pvParameters) {
  uint32_t lastReconnectAttemptWiFi = millis();
  uint32_t lastReconnectAttemptGSM = millis();

  while (true) {
    uint32_t now = millis();

    if (wifiConnected) {
      if (!mqttWiFi.connected()) {
        isMQTTConnectedWiFi = false;
        if (now - lastReconnectAttemptWiFi > 10000) {
          lastReconnectAttemptWiFi = now;
          if (mqttConnect(mqttWiFi, WIFI_CONNECTED)) {
            isMQTTConnectedWiFi = true;
          }
        }
      } else {
        isMQTTConnectedWiFi = true;
        lastReconnectAttemptWiFi = now;
        mqttWiFi.loop();
      }
    }

    

    if (gsmConnected) {
      if (!mqttGSM.connected()) {
        isMQTTConnectedGSM = false;
        if (now - lastReconnectAttemptGSM > 10000) {
          lastReconnectAttemptGSM = now;
          if (mqttConnect(mqttGSM, GSM_CONNECTED)) {
            isMQTTConnectedGSM = true;
            
          }
        }
      } else {
        isMQTTConnectedGSM = true;
        lastReconnectAttemptGSM = now;
        mqttGSM.loop();
      }
    }


    networkInfo.mqttWiFiConnected = isMQTTConnectedWiFi;
    networkInfo.mqttGSMConnected = isMQTTConnectedGSM;
    networkInfo.wifiConnected = wifiConnected;
    networkInfo.gprsConnected = networkInfo.mqttGSMConnected;

    // Update network Connectivity
    
  
    display.writeNum("p2.pic", networkInfo.wifiConnected ? WIFI_OK : WIFI_NOK);
    display.writeNum("p1.pic", (networkInfo.mqttWiFiConnected || networkInfo.mqttGSMConnected) ? MQTT_OK : MQTT_NOK);
    display.writeNum("p3.pic", networkInfo.mqttGSMConnected ? GPRS_OK : GPRS_NOK);
    // if(networkInfo.gprsConnected){
    //   display.writeNum("p3.pic", 10);
    // }
    // else{
    //   display.writeNum("p3.pic", 9);
    // }

    Serial.println("GPRS:" + String(networkInfo.gprsConnected));
    Serial.println("WIFI:" + String(networkInfo.wifiConnected));
    Serial.println("MQTT:" + String(networkInfo.mqttWiFiConnected || networkInfo.mqttGSMConnected));
    // Serial.println(" MQTT GSM:" + String(networkInfo.mqttGSMConnected));

    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void startNetworkMqttManagerTask(int pwrPin, int led, NetworkMode mode) {
  powerPin = pwrPin;
  ledPin = led;

  Serial.println("Starting Network MQTT Manager...");

  if (mode == WIFI_ONLY || mode == WIFI_GSM) {
    Serial.println("📡 WiFi Mode Enabled");
    xTaskCreatePinnedToCore(wifiTask, "WiFi Task", 4096, NULL, 2, NULL, 0);
  }

  if (mode == GSM_ONLY || mode == WIFI_GSM) {
    Serial.println("📶 GSM Mode Enabled");
    xTaskCreatePinnedToCore(gsmTask, "GSM Task", 4096, NULL, 2, NULL, 1);
  }

  Serial.println("⚙️ Starting MQTT Task...");
  xTaskCreatePinnedToCore(mqttTask, "MQTT Task", 4096, NULL, 1, NULL, 1);
}
