#pragma once

// Including required libraries
#include <Arduino.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

// Defining settings for LoRa communication
#define RF95_FREQ 915.0            // LoRa Frequency in MHz
#define RFM95_CS 16                // Chip Select Pin for LoRa
#define RFM95_INT 21               // Interrupt Pin for LoRa
#define RFM95_RST 17               // Reset Pin for LoRa
#define TX_POWER 23                // Transmit Power for LoRa in dBm
#define BANDWIDTH 125              // Bandwidth for LoRa in kHz
#define SPREADING_FACTOR 7         // Spreading Factor for LoRa
#define CODING_RATE 5              // Coding Rate for LoRa (4/x)
#define PREAMBLE_LENGTH 8          // Preamble Length for LoRa

// Defining settings for BNO055 I2C communication
#define BNO055_SDA_PIN 4           // SDA pin for BNO055
#define BNO055_SCL_PIN 5           // SCL pin for BNO055

// Externally declaring the LoRa radio object for use in other files
extern RH_RF95 rf95;

// Function prototypes for initializing and operating the radio and BNO055 sensor
void initRadio();
void receiveAndReply();
void initBNO055();
void logBNO055Data();
