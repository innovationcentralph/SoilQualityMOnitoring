#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

// Debug flags
#define DEBUG_WIFI
#define DEBUG_MQTT
#define DEBUG_SENSORS

// Watchdog timer timeout (in milliseconds)
#define WDT_TIME_MS 30000

#define MQTT_DISCONNECT_TIMEOUT_MS 300000

#define EEPROM_SIZE 512
#define EEPROM_SEND_INTERVAL_ADDR 0

// MQTT configuration
extern const char* mqttServer;
extern const int mqttPort;
extern const char* mqttUser;
extern const char* mqttPassword;
extern const char* deviceESN;

// Pin Definition
#define MASTER_RO 16
#define MASTER_DI 17
#define MASTER_EN 5




// Nextion Assets

#define TEMPERATURE_LABEL 4
#define HUMIDITY_LABEL    5
#define WIFI_OK           1
#define WIFI_NOK          7
#define SERVER_OK         0
#define SERVER_NOK        6

#endif // SYSTEM_CONFIG_H

