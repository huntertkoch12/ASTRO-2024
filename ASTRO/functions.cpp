
// Include the configuration settings
#include "config.h"

// Include the RadioHead RF95 library for LoRa communication
#include <RH_RF95.h>

// Initialize the LoRa radio object with the Chip Select and Interrupt pins
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Function to initialize the LoRa radio
void initRadio() {
  // Set up the LED and Reset pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Initialize serial communication with a baud rate of 115200
  Serial.begin(115200);
  while (!Serial) delay(1);  // Wait for serial port to connect (needed for Leonardo only)
  delay(100);

  // Print a start-up message
  Serial.println("Feather LoRa RX Test!");

  // Perform a manual reset of the LoRa radio
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Initialize the LoRa radio
  if (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);  // Infinite loop to indicate failure
  }
  Serial.println("LoRa radio init OK!");

  // Set the frequency for the LoRa radio
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency Failed!");
    while (1);  // Infinite loop to indicate failure
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  // Configure the LoRa radio settings based on the parameters defined in config.h
  rf95.setTxPower(TX_POWER, false);
  rf95.setSignalBandwidth(BANDWIDTH * 1000);
  rf95.setSpreadingFactor(SPREADING_FACTOR);
  rf95.setCodingRate4(CODING_RATE);
  rf95.setPreambleLength(PREAMBLE_LENGTH);
}

// Function to receive a message and reply back
void receiveAndReply() {
  // Check if there is a message available to receive
  if (rf95.available()) {
    // Buffer to hold the incoming message
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    // Attempt to receive the message
    if (rf95.recv(buf, &len)) {
      digitalWrite(LED_BUILTIN, HIGH);  // Turn on the LED to indicate reception

      // Print the received message to the Serial Monitor
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char*)buf);
      Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);
      
      // Print the SNR of the last received message
      Serial.print("SNR: ");
      Serial.println(rf95.lastSNR(), DEC);

      // Prepare a reply message
      uint8_t data[] = "And hello back to you!";
      
      // Send the reply message
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Sent a reply!");
      
      digitalWrite(LED_BUILTIN, LOW);  // Turn off the LED
    } else {
      // Print an error message if reception failed
      Serial.println("Receive failed!");
    }
  }
}
