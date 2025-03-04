#include "SensorManager.h"
#include "DFRobot_SHT20.h"
#include <ModbusRTU.h>
#include "SystemConfig.h"

ModbusRTU modbus;

// Define the task handle
TaskHandle_t xTaskHandle_SensorManager = NULL;
SemaphoreHandle_t xSemaphore_Sensor = NULL;
bool sensorDataReady = false;

// Define the global sensor data
SensorData_t sensorData = { 0.0, 0.0, 0, 0, 0, 0, 0, 0 };

DFRobot_SHT20 sht20(&Wire, SHT20_I2C_ADDR);

// Timer interval for reading sensors (in milliseconds)
const uint32_t SENSOR_READ_INTERVAL_MS = 5000;

void modbusTask(void* pvParameters);
void SensorManagerTask(void* pvParameters);

void startSensorManagerTask() {

  xSemaphore_Sensor = xSemaphoreCreateMutex();
  if (xSemaphore_Sensor == NULL) {
    Serial.println("Failed to create mutex for sensor");
    return;
  }

  xTaskCreatePinnedToCore(
    SensorManagerTask,           // Task function
    "Sensor Manager",            // Task name
    4096,                        // Stack size
    NULL,                        // Task parameters
    1,                           // Task priority
    &xTaskHandle_SensorManager,  // Task handle
    1                            // Core ID
  );

  // Create a task to handle Modbus communication
  xTaskCreatePinnedToCore(
    modbusTask,    // Function to run
    "ModbusTask",  // Task name
    4096,          // Stack size (in bytes)
    NULL,          // Task parameter
    1,             // Priority
    NULL,          // Task handle
    1              // Core ID (1 for ESP32 dual-core)
  );
}

// Modbus task function
void modbusTask(void* pvParameters) {
  for (;;) {

    // Handle Modbus tasks
    modbus.task();

    // Small delay to prevent CPU hogging
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

void SensorManagerTask(void* pvParameters) {
#ifdef DEBUG_SENSORS
  Serial.print("Sensor Manager running on core ");
#endif
  Serial.println(xPortGetCoreID());

// Initialize sensor peripherals here
#ifdef DEBUG_SENSORS
  Serial.println("Initializing sensors...");
#endif

  // Initialize Modbus Port as Master
  Serial2.begin(9600, SERIAL_8N1, MASTER_RO, MASTER_DI);
  modbus.begin(&Serial2, MASTER_EN);
  modbus.setBaudrate(9600);
  modbus.master();

#ifdef DEBUG_SENSORS
  Serial.println("Modbus Master Initialized");
#endif

  uint32_t lastReadTime = -1 * SENSOR_READ_INTERVAL_MS;  // to read ASAP

  // Init SHT20 Sensor
  sht20.initSHT20();

  // Main task loop
  for (;;) {
    if (millis() - lastReadTime >= SENSOR_READ_INTERVAL_MS) {
      // Time to read sensors
      Serial.println("Reading sensors...");

      if (xSemaphoreTake(xSemaphore_Sensor, portMAX_DELAY) == pdTRUE) {

        // read temperature and humidity
        sensorData.temperature = sht20.readTemperature();
        sensorData.humidity = sht20.readHumidity();

#ifdef DEBUG_SENSORS
        Serial.print("Temperature: ");
        Serial.println(sensorData.temperature);
        Serial.print("Humidity: ");
        Serial.println(sensorData.humidity);

#endif

        // read moisture, soil temperature, ph, n, p, k

        uint16_t modbus_resp[16];
        bool res = modbus.readHreg(1, 0x0000, modbus_resp, 8, nullptr);

        vTaskDelay(2000);  // Example delay

        if (res == true) {
          sensorData.soil_moisture = modbus_resp[0] / 10.0;
          sensorData.soil_temperature = modbus_resp[1] / 10.0;
          sensorData.soil_ph = modbus_resp[3] / 10.0;
          sensorData.soil_nitrogen = modbus_resp[4];
          sensorData.soil_phosphorus = modbus_resp[5];
          sensorData.soil_potassium = modbus_resp[6];

#ifdef DEBUG_SENSORS
          Serial.println("   Soil Moisture: " + String(sensorData.soil_moisture) + "%");
          Serial.println("Soil Temperature: " + String(sensorData.soil_temperature) + "°C");
          Serial.println("        pH Level: " + String(sensorData.soil_ph));
          Serial.println("               N: " + String(sensorData.soil_nitrogen) + "mg/kg");
          Serial.println("               P: " + String(sensorData.soil_phosphorus) + "mg/kg");
          Serial.println("               K: " + String(sensorData.soil_potassium) + "mg/kg");
#endif
        } else {
          Serial.println("Error reading NPK sensor");
        }

        xSemaphoreGive(xSemaphore_Sensor);
        sensorDataReady = res ? true : false; // -----------------------------> Add checking for SHT Sensor validity
      }

      // Reset timer
      lastReadTime = millis();
    }

    // Task delay to yield execution
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}
