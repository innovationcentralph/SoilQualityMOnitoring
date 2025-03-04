#ifndef I2C_LOGGER_H
#define I2C_LOGGER_H

#include <Arduino.h>
#include <Wire.h>

class I2CLogger {
public:
    I2CLogger(uint8_t slaveAddress); // Constructor



    // File Operations
    bool createFile(const char* filename);
    bool deleteFile(const char* filename);
    bool selectFile(const char* filename);
    bool saveLogToFile(const char* filename, const char* logData);
    bool appendLogData(const char* logData);

    // RTC Operations
    bool setRTC(uint32_t epochTime);
    uint32_t getRTC(); // Return epoch time or 0xFFFFFFFF on failure

private:
    uint8_t slaveAddr; // I2C slave address

    bool sendCommand(uint8_t command, uint8_t* payload = nullptr, size_t length = 0);
};

#endif
