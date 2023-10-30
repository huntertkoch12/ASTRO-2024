#pragma once

//*******************************************//
//             Include Libraries             //
//*******************************************//

#include <Arduino.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <SdFat.h>
#include <SPI.h>

//*******************************************//
//             LoRa Settings                 //
//*******************************************//

#define RF95_FREQ 915.0            // LoRa Frequency (MHz)
#define RFM95_CS 16                // Chip Select Pin (LoRa)
#define RFM95_INT 21               // Interrupt Pin (LoRa)
#define RFM95_RST 17               // Reset Pin (LoRa)
#define TX_POWER 23                // Transmit Power (LoRa, dBm)
#define BANDWIDTH 125              // Bandwidth (LoRa, kHz)
#define SPREADING_FACTOR 7         // Spreading Factor (LoRa)
#define CODING_RATE 5              // Coding Rate (LoRa, 4/x)
#define PREAMBLE_LENGTH 8          // Preamble Length (LoRa)

//*******************************************//
//             BNO055 Settings               //
//*******************************************//

#define BNO055_SDA_PIN 4           // SDA pin (BNO055)
#define BNO055_SCL_PIN 5           // SCL pin (BNO055)

//*******************************************//
//           External Declarations           //
//*******************************************//

extern RH_RF95 rf95;              // LoRa Radio Object
extern SdFat sd;                  // SD Card Object
extern SdFile dataFile;           // Data File Object

//*******************************************//
//         Function Prototypes               //
//*******************************************//

// LoRa Radio Functions
void initRadio();                 // Initialize LoRa Radio
void receiveAndReply();           // Receive Message & Send Reply

// BNO055 Sensor Functions
void initBNO055();                // Initialize BNO055 Sensor
void logBNO055Data();             // Log Data from BNO055 Sensor

// SD Card Functions
void setupSDCard();               // Initialize SD Card
void logDataToSD();               // Log Data to SD Card
