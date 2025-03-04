#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include <Arduino.h>

// Struct to hold sensor data
typedef struct {
    float temperature; 
    float humidity;    
    float soil_moisture;
    float soil_temperature;
    float soil_ph;
    uint16_t soil_nitrogen;
    uint16_t soil_phosphorus;
    uint16_t soil_potassium;
} SensorData_t;

// RTOS Task Handle
extern TaskHandle_t xTaskHandle_SensorManager;
extern SemaphoreHandle_t xSemaphore_Sensor;
extern bool sensorDataReady;

// Global Sensor Data
extern SensorData_t sensorData;

// Function Prototypes
void startSensorManagerTask();  // Function to start the task
void SensorManagerTask(void* pvParameters); // Task function

#endif // SENSOR_MANAGER_H
