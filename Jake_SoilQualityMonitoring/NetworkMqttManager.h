#ifndef NETWORK_MQTT_MANAGER_H
#define NETWORK_MQTT_MANAGER_H

#include <PubSubClient.h>

typedef enum {
  WIFI_ONLY,
  GSM_ONLY,
  WIFI_GSM
}NetworkMode;

// Struct to hold network information
typedef struct {
    bool wifiConnected;       // Connection Status
    bool gprsConnected;       // GSM Conneciton Status
    bool mqttGSMConnected;       // Connection to MQTT Server
    bool mqttWiFiConnected;       // Connection to MQTT Server
    String SSID;              // Current SSID
    String password;          // Password (optional, based on use case)
    int RSSI;                 // Signal strength (RSSI)
} NetworkInfo_t;

// Declare the global NetworkInfo object as extern
extern NetworkInfo_t networkInfo;

void startNetworkMqttManagerTask(int powerPin, int ledPin = -1, NetworkMode mode = WIFI_GSM);

extern bool isMQTTConnectedWiFi;
extern bool isMQTTConnectedGSM;
extern PubSubClient mqttWiFi;
extern PubSubClient mqttGSM;

#define WIFI_CONNECTED  1
#define GSM_CONNECTED   2

#endif // NETWORK_MQTT_MANAGER_H