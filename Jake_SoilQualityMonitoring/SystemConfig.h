#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include "EasyNextionLibrary.h"

// Debug flags
#define DEBUG_WIFI
#define DEBUG_MQTT
#define DEBUG_MQTT_PAYLOAD
#define DEBUG_GPRS
#define DEBUG_SENSORS
#define DEBUG_SYSTEM

// Watchdog timer timeout (in milliseconds)
#define WDT_TIME_MS 30000

#define MQTT_DISCONNECT_TIMEOUT_MS 120000

#define EEPROM_SIZE 512
#define EEPROM_SEND_INTERVAL_ADDR 0

// MQTT configuration
extern const char* mqttServer;
extern const int mqttPort;
extern const char* mqttUser;
extern const char* mqttPassword;
extern const char* deviceESN;

extern const char* topicInit;
extern const char* topicConfig;
extern const char* topicSensorData;


// Wi-Fi credentials
extern const char* wifiSSID;
extern const char* wifiPass;

// Display 
extern EasyNex display;






// Pin Definition
#define MASTER_RO 26
#define MASTER_DI 25
#define MASTER_EN 27

#define TIMER_DONE 15
#define LED_STAT_PIN 12

#define GSM_PWR 4
#define NETWORK_LED 13 // LED Indicator pin for MQTT connection

#define LED_ON  HIGH
#define LED_OFF LOW




// Nextion Assets

#define GPRS_OK    10
#define GPRS_NOK   9
#define WIFI_OK    8
#define WIFI_NOK   7
#define MQTT_OK    6
#define MQTT_NOK   5

#endif // SYSTEM_CONFIG_H

