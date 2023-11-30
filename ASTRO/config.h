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
#include <Adafruit_GPS.h>
#include <Adafruit_NeoPixel.h>
#include <RTClib.h>

//*******************************************//
//             LoRa Settings                 //
//*******************************************//

#define RF95_FREQ 921.0            // LoRa Frequency (MHz)
#define RFM95_CS   16
#define RFM95_INT  21
#define RFM95_RST  17
#define TX_POWER 23                // Transmit Power (LoRa, dBm)
#define BANDWIDTH 125              // Bandwidth (LoRa, kHz)
#define SPREADING_FACTOR 12         // Spreading Factor (LoRa)
#define CODING_RATE 8              // Coding Rate (LoRa, 4/x)
#define PREAMBLE_LENGTH 12          // Preamble Length (LoRa)

//*******************************************//
//             GPS Settings                  //
//*******************************************//

#define GPS_RX_PIN 18  // Connect to the TX pin of the GPS module
#define GPS_TX_PIN 19  // Connect to the RX pin of the GPS module

//*******************************************//
//           External Declarations           //
//*******************************************//

extern RH_RF95 rf95;              // LoRa Radio Object
extern SdFat sd;                  // SD Card Object
extern SdFile dataFile;           // Data File Object
extern Adafruit_MPL3115A2 baro;   // Altimeter Object
extern Adafruit_NeoPixel strip;
extern RTC_DS3231 rtc;

//*******************************************//
//                 NEOPIXEL                  //
//*******************************************//
#define NEOPIXEL_PIN PIN_NEOPIXEL   // GPIO4 is used for the onboard NeoPixel
#define NEOPIXEL_NUM 1   // Number of NeoPixels

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

// GPS Functions
void initGPS();
void transmitGPSData();
void rawGPS();
void setGPSUpdateRate();
void setGPSUpdateRate(int milliseconds);

// RTC Functions
void setupRTC();

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
