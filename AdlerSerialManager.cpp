#include "SerialManager.h"

// Constructor implementation
SerialManager::SerialManager(HardwareSerial* serialPort) {
    serial = serialPort;
}

// Compute checksum implementation
uint32_t SerialManager::computeChecksum(uint8_t functionID, uint8_t flags, uint8_t length, const uint8_t* data, size_t dataLength) {
    uint32_t s1 = 1;
    uint32_t s2 = 0;
    const uint32_t constant = 65521;

    // Process the initial variables
    s1 = (s1 + startBytes[0]) % constant;
    s2 = (s2 + s1) % constant;

    s1 = (s1 + startBytes[1]) % constant;
    s2 = (s2 + s1) % constant;

    s1 = (s1 + functionID) % constant;
    s2 = (s2 + s1) % constant;

    s1 = (s1 + flags) % constant;
    s2 = (s2 + s1) % constant;

    s1 = (s1 + length) % constant;
    s2 = (s2 + s1) % constant;

    for (size_t i = 0; i < dataLength; i++) {
        s1 = (s1 + data[i]) % constant;
        s2 = (s2 + s1) % constant;
    }

    return (s2 << 16) | s1;
}

// Send message implementation
void SerialManager::sendMessage(uint8_t functionID, uint8_t flags, const String& payload) {
    uint8_t length = payload.length();
    uint8_t data[length];
    payload.getBytes(data, length + 1); // Copy payload to byte array

    uint32_t checksum = computeChecksum(functionID, flags, length, data, length);
    serial->write(startBytes[0]);
    serial->write(startBytes[1]);
    serial->write(functionID);
    serial->write(flags);
    serial->write(length);
    serial->write(data, length);
    serial->write((uint8_t*)&checksum, 4); // Write checksum bytes
}

// Receive message implementation
bool SerialManager::receiveMessage(uint8_t& functionID, uint8_t& flags, String& payload) {
    if (serial->available() >= 7) {
        if (serial->read() == 0xAA && serial->read() == 0xBB) {
            functionID = serial->read();
            flags = serial->read();
            uint8_t length = serial->read();

            uint8_t data[length];
            serial->readBytes(data, length);

            uint32_t receivedChecksum;
            serial->readBytes((uint8_t*)&receivedChecksum, 4);

            uint32_t computedChecksum = computeChecksum(functionID, flags, length, data, length);
            if (receivedChecksum == computedChecksum) {
                payload = String((char*)data);
                return true;
            } else {
                Serial.println("Checksum error!");
                return false;
            }
        }
    }
    return false;
}
