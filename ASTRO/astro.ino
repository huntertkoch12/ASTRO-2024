#include "config.h"

// Declaration of external functions from other files or libraries
extern void initRadio();
extern void receiveAndReply();
extern void initBNO055();
extern void logBNO055Data();

void setup() {
  // Initialize serial communication at 115200 baud rate
  Serial.begin(115200);
  while (!Serial) {
    delay(1); // Wait for the serial connection to be established
  }
  
  Serial.println("Initialization on Core 0!");
  
  // Initialize the BNO055 sensor
  initBNO055();
}

void loop() {
  // Continuously log BNO055 sensor data
  logBNO055Data();
  
}

void setup1() {
  // Initialize the LoRa radio
  initRadio();
}

void loop1() {
  // Continuously check for incoming messages and respond
  receiveAndReply();
}
