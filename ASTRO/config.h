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
#include <Adafruit_MPL3115A2.h>

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
//           External Declarations           //
//*******************************************//

extern RH_RF95 rf95;              // LoRa Radio Object
extern SdFat sd;                  // SD Card Object
extern SdFile dataFile;           // Data File Object
extern Adafruit_MPL3115A2 baro;   // Altimeter Object

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

// Altimeter Functions
void initMPL3115A2();
void logMPL3115A2Data();

/*
 * RP2040 Pin Configuration for Sensors
 *
 * I2C Configuration for BNO055 and MPL3115A2:
 * - SDA (Serial Data Line): Connect to GPIO2 (SDA)
 * - SCL (Serial Clock Line): Connect to GPIO3 (SCL)
 *
 * SPI Configuration for SD Card:
 * - MISO (Master In Slave Out): Connect to GPIO8 (MISO)
 * - MOSI (Master Out Slave In): Connect to GPIO15 (MOSI)
 * - SCK (Serial Clock): Connect to GPIO14 (SCK)
 * - CS (Chip Select): Connect to GPIO10 (D10)
 *
 * LoRa (RFM95) Configuration:
 * - CS (Chip Select): Connect to GPIO16 (RFM95_CS)
 * - RST (Reset): Connect to GPIO17 (RFM95_RST)
 * - Other specific LoRa pins as defined in the LoRa library and configuration
 *
 * UART Configuration for Serial Communication:
 * - TX (Transmit): Connect to GPIO0 (TX)
 * - RX (Receive): Connect to GPIO1 (RX)
 */
