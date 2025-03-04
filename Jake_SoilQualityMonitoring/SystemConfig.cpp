#include "SystemConfig.h"

// Wi-Fi credentials
const char* wifiSSID = "PLDTinnov";
const char* wifiPass = "Password12345!";

// MQTT configuration variables
const char* mqttServer     = "3.27.210.100";
const int mqttPort         = 1883;
const char* mqttUser       = "mqtt";
const char* mqttPassword   = "ICPHmqtt!";
const char* deviceESN      = "SQM001";  // Not Used for now

// MQTT broker
const char* topicInit       = "SQM/SQM-001/init";
const char* topicConfig     = "SQM/SQM-001/config";
const char* topicSensorData = "SQM/SQM-001";


// Nextion HMI Display instance
EasyNex display(Serial);

