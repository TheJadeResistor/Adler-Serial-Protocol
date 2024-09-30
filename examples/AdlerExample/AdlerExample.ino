#include <HardwareSerial.h>
#include "serialManager.h"
#include <AdlerSerialManager.h>

HardwareSerial MaixCamSerial(2); // use UART2
SerialManager uart(&MaixCamSerial);

#define RX_pin 16 // uart1
#define TX_pin 17 // uart1

void setup() {
  Serial.begin(115200);
  MaixCamSerial.begin(115200, SERIAL_8N1, RX_pin, TX_pin);
}

void loop() {
  // Example usage for sending
  String myString = "hello world";
  uart.sendMessage(0x01, 0x02, myString);

  // Example usage for receiving
  uint8_t functionID, flags;
  String payload;
  if (uart.receiveMessage(functionID, flags, payload)) {
    Serial.print("Received message: ");
    Serial.print(payload);
    Serial.println("|end|");
  }

  delay(1000); // Add delay for demonstration purpose
}




