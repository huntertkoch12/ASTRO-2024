#include <Arduino.h>
#include <Wire.h>
#include <FreeRTOS.h>
#include "config.h"
#include "functions.ino"

void setup() {
  Serial.begin(9600);
  Wire.setSDA(SDA_PIN);
  Wire.setSCL(SCL_PIN);
  Wire.begin();

  setupLoRa();
  setupBNO055();
  setupMPL3115A2();
  setupSDCard();
  setupGPS();

  // Create a task for continuous LoRa broadcasting on core 1
  xTaskCreatePinnedToCore(
    loRaTask,   // Function to implement the task
    "LoRaTask", // Name of the task
    10000,      // Stack size in words
    NULL,       // Task input parameter
    1,          // Priority of the task
    NULL,       // Task handle.
    1           // Core where the task should run
  );
}

void loop() {
  // Main loop running on core 0
  readBNO055();
  readMPL3115A2();
  readGPS();
  logDataToSDCard("Sample log data");
  delay(1000);
}

// Task for continuous LoRa broadcasting
void loRaTask(void *parameter) {
  for (;;) {
    sendLoRaMessage("Hello, LoRa!");
    delay(1000);
  }
}
