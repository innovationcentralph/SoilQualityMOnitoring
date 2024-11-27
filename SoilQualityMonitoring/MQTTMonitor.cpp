#include "MQTTMonitor.h"

// Include external dependencies
#include "MQTTHandler.h"
#include "NetworkManager.h"
#include "WiFi.h"
#include "EEPROM.h"


// Define task handle
TaskHandle_t xTaskHandle_MQTTMonitor = NULL;
SemaphoreHandle_t xSemaphore_LocalTime = NULL;



// Create an instance of the MQTT handler
extern MQTTHandler mqttHandler;  // Assuming the instance is declared in the main sketch

String localTime = "";
unsigned long epochTime = 0;

// Function prototypes
String getTime12HourFormat(unsigned long epoch, int gmtOffset);
void writeSendIntervalToEEPROM(uint32_t interval);


// Function to start the MQTT monitor task
void startMQTTMonitorTask() {

  xSemaphore_LocalTime = xSemaphoreCreateMutex();
  if (xSemaphore_LocalTime == NULL) {
      Serial.println("Failed to create mutex for localTime");
      return;
  }

  xTaskCreatePinnedToCore(
    MQTTMonitor_Routine,       // Task function
    "MQTT Monitor",            // Task name
    4096,                      // Stack size
    NULL,                      // Task parameters
    1,                         // Task priority
    &xTaskHandle_MQTTMonitor,  // Task handle
    0                          // Core ID
  );
}

// MQTT monitoring routine
void MQTTMonitor_Routine(void* pvParameters) {
  Serial.print("MQTT monitoring running on core ");
  Serial.println(xPortGetCoreID());


  for (;;) {
     if (networkInfo.wifiConnected == true){
   
      // Check MQTT connectivity
      if (mqttHandler.checkConnectivity()) {
        //Serial.println("MQTT Connected");
      } else {
        //Serial.println("MQTT Disconnected");
      }

      // Check for incoming messages
      if (mqttHandler.messageAvailable()) {
        String incomingTopic = mqttHandler.getMessageTopic();
        String incomingPayload = mqttHandler.getMessagePayload();

        // Process the message

        Serial.print("Received message on topic: ");
        Serial.println(incomingTopic);
        Serial.print("Message payload: ");
        Serial.println(incomingPayload);

        if(incomingTopic.equals("Innov/GiveTime")){
          Serial.println("EPOCH TIME: " + incomingPayload);  
          epochTime = strtoul(incomingPayload.c_str(), NULL, 10);
          if (xSemaphoreTake(xSemaphore_LocalTime, portMAX_DELAY) == pdTRUE) {
               localTime = getTime12HourFormat(strtoul(incomingPayload.c_str(), NULL, 10), 8); // Philippines is GMT + 8
              Serial.println("Local Time: " + localTime);
              xSemaphoreGive(xSemaphore_LocalTime);
          }              
         
        }

        // Process change in sending interval
        if (incomingTopic.equals("SQM/SQM-001/interval")) {
            // Parse the payload for a new interval
            uint32_t newInterval = incomingPayload.toInt();
            if (newInterval > 10000 && newInterval <= 3600000) { // Ensure valid interval > 10s
                sendInterval = newInterval;
                writeSendIntervalToEEPROM(newInterval); // Save to EEPROM
                Serial.println("Updated send interval: " + String(sendInterval) + " ms");
            }
        }


        // Clear the message flag
        mqttHandler.clearMessageFlag();
      }
    }

    // Yield to other tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

// Function to convert epoch time and GMT to a 12-hour format time string
String getTime12HourFormat(unsigned long epoch, int gmtOffset) {
  // Adjust for GMT offset
  epoch += gmtOffset * 3600;

  // Calculate hours, minutes, and seconds
  unsigned long secondsInDay = epoch % 86400; // Seconds since midnight
  int hours = secondsInDay / 3600;
  int minutes = (secondsInDay % 3600) / 60;
  int seconds = secondsInDay % 60;

  // Convert to 12-hour format
  String period = (hours >= 12) ? "PM" : "AM";
  if (hours == 0) {
    hours = 12; // Midnight
  } else if (hours > 12) {
    hours -= 12; // Convert to 12-hour format
  }

  // Create a formatted time string
  char timeString[12]; // Format: HH:MM:SS AM/PM
  sprintf(timeString, "%02d:%02d %s", hours, minutes, period.c_str());

  return String(timeString);
}


// Write `sendInterval` to EEPROM
void writeSendIntervalToEEPROM(uint32_t interval) {
    EEPROM.put(EEPROM_SEND_INTERVAL_ADDR, interval);
    EEPROM.commit();
}
