#pragma once

#include <HardwareSerial.h>

class SerialManager {
public:
    // Constructor
    SerialManager(HardwareSerial* serialPort);

    // Function to compute checksum
    uint32_t computeChecksum(uint8_t functionID, uint8_t flags, uint8_t length, const uint8_t* data, size_t dataLength);

    // Function to send a message
    void sendMessage(uint8_t functionID, uint8_t flags, const String& payload);

    // Function to receive a message
    bool receiveMessage(uint8_t& functionID, uint8_t& flags, String& payload);

private:
    HardwareSerial* serial;
    uint8_t startBytes[2] = {0xAA, 0xBB};
};
