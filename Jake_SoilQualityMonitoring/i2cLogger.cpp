#include "i2cLogger.h"

// Command IDs
#define GET_TIME 0x01
#define SET_TIME 0x02
#define SAVE_LOGS_TO_FILE 0x03
#define CREATE_FILE 0x04
#define DELETE_FILE 0x05
#define SELECT_FILE 0x06
#define APPEND_LOG_DATA 0x07

// Constructor
I2CLogger::I2CLogger(uint8_t slaveAddress) : slaveAddr(slaveAddress) {}

// Helper function to send I2C commands
bool I2CLogger::sendCommand(uint8_t command, uint8_t* payload, size_t length) {
    Wire.beginTransmission(slaveAddr);
    Wire.write(command); // Send command ID
    if (payload != nullptr && length > 0) {
        Wire.write(payload, length); // Send payload if any
    }
    if (Wire.endTransmission() == 0) { // Check for successful transmission
        return true;
    }
    return false;
}

// File Operations
bool I2CLogger::createFile(const char* filename) {
    Serial.print("Requesting file creation for: ");
    Serial.println(filename);

    if (sendCommand(CREATE_FILE, (uint8_t*)filename, strlen(filename))) {
        Serial.println("File creation request sent.");
        return true;
    }
    Serial.println("File creation failed.");
    return false;
}

bool I2CLogger::deleteFile(const char* filename) {
    Serial.print("Requesting file deletion for: ");
    Serial.println(filename);

    if (sendCommand(DELETE_FILE, (uint8_t*)filename, strlen(filename))) {
        Serial.println("File deletion request sent.");
        return true;
    }
    Serial.println("File deletion failed.");
    return false;
}

bool I2CLogger::selectFile(const char* filename) {
    Serial.print("Selecting file on slave: ");
    Serial.println(filename);

    if (sendCommand(SELECT_FILE, (uint8_t*)filename, strlen(filename))) {
        Serial.println("File selection request sent.");
        return true;
    }
    Serial.println("File selection failed.");
    return false;
}

bool I2CLogger::saveLogToFile(const char* filename, const char* logData) {
    Serial.print("Sending log data to slave for file: ");
    Serial.println(filename);

    uint8_t payload[32];
    size_t filenameLength = strlen(filename);
    size_t logDataLength = strlen(logData);

    if (filenameLength > 12 || filenameLength + logDataLength > 31) {
        Serial.println("Payload exceeds I2C buffer size!");
        return false;
    }

    memset(payload, 0, sizeof(payload));
    memcpy(payload, filename, filenameLength);
    strcpy((char*)(payload + 12), logData);

    if (sendCommand(SAVE_LOGS_TO_FILE, payload, filenameLength + logDataLength)) {
        Serial.println("Log data sent.");
        return true;
    }
    Serial.println("Failed to send log data.");
    return false;
}

bool I2CLogger::appendLogData(const char* logData) {
    Serial.print("Appending log data to selected file: ");
    Serial.println(logData);

    size_t logDataLength = strlen(logData);

    if (logDataLength > 31) {
        Serial.println("Log data exceeds I2C buffer size!");
        return false;
    }

    if (sendCommand(APPEND_LOG_DATA, (uint8_t*)logData, logDataLength)) {
        Serial.println("Log data sent.");
        return true;
    }
    Serial.println("Failed to append log data.");
    return false;
}

// RTC Operations
bool I2CLogger::setRTC(uint32_t epochTime) {
    Serial.print("Setting RTC on slave with epoch time: ");
    Serial.println(epochTime);

    uint8_t payload[4];
    payload[0] = (epochTime >> 24) & 0xFF;
    payload[1] = (epochTime >> 16) & 0xFF;
    payload[2] = (epochTime >> 8) & 0xFF;
    payload[3] = epochTime & 0xFF;

    if (sendCommand(SET_TIME, payload, 4)) {
        Serial.println("RTC set request sent.");
        return true;
    }
    Serial.println("Failed to set RTC.");
    return false;
}

uint32_t I2CLogger::getRTC() {
    Serial.println("Requesting epoch time from slave...");

    if (!sendCommand(GET_TIME)) {
        Serial.println("Failed to request time.");
        return 0xFFFFFFFF; // Indicate failure
    }

    Wire.requestFrom(slaveAddr, 4);
    if (Wire.available() == 4) {
        uint32_t epochTime = 0;
        epochTime |= (Wire.read() << 24); // MSB
        epochTime |= (Wire.read() << 16);
        epochTime |= (Wire.read() << 8);
        epochTime |= Wire.read();        // LSB

        Serial.print("Epoch Time received: ");
        Serial.println(epochTime);

        return epochTime; // Return the epoch time
    }

    Serial.println("Failed to receive epoch time from slave.");
    return 0xFFFFFFFF; // Indicate failure
}
