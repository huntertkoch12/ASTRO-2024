#pragma once

#include <Arduino.h>
#include <RH_RF95.h>

// LoRa Settings
#define RF95_FREQ 915.0     // Frequency in MHz
#define RFM95_CS 16         // Chip Select Pin
#define RFM95_INT 21        // Interrupt Pin
#define RFM95_RST 17        // Reset Pin
#define TX_POWER 23         // Transmit Power in dBm
#define BANDWIDTH 125       // Bandwidth in kHz
#define SPREADING_FACTOR 7  // Spreading Factor
#define CODING_RATE 5       // Coding Rate (4/x)
#define PREAMBLE_LENGTH 8   // Preamble Length

extern RH_RF95 rf95;

void initRadio();
void receiveAndReply();
