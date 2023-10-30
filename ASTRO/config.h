// config.h
#ifndef CONFIG_H
#define CONFIG_H

// LoRa Configuration
const int RFM_CS_PIN = 8;   // Change to the correct pin for Chip Select
const int RFM_RST_PIN = 4;  // Change to the correct pin for Reset
const int RFM_DIO0 = 21;
const int RFM_DIO1 = 22;
const int RFM_DIO2 = 23;
const int RFM_DIO3 = 19;
const int RFM_DIO4 = 20;
const int RFM_DIO5 = 18;

// LoRa Signal Configuration
const float LORA_FREQUENCY = 915.0; // Frequency in MHz
const int LORA_TX_POWER = 14;       // Transmission power in dBm
const RH_RF95::ModemConfigChoice LORA_MODEM_CONFIG = RH_RF95::Bw125Cr45Sf128; // Modem configuration
const uint16_t LORA_PREAMBLE_LENGTH = 8;   // Preamble length in bytes
const bool LORA_PROMISCUOUS_MODE = false;  // Promiscuous mode setting
const uint8_t LORA_CODING_RATE = 5;        // Coding rate (valid values are between 5 and 8)
const uint8_t LORA_SPREADING_FACTOR = 7;   // Spreading factor (valid values are between 6 and 12)


// BNO055 Configuration
const int BNO055_INT_PIN = -1; // Change to the correct pin if you are using interrupts

// MPL3115A2 Configuration
const int MPL3115A2_INT_PIN = -1; // Change to the correct pin if you are using interrupts

// SD Card Configuration
const int SD_CS_PIN = 10;  // Change to the correct pin for Chip Select of SD Card


// GPS Configuration
const int GPS_RX_PIN = 3;  // Change to the correct pin
const int GPS_TX_PIN = 4;  // Change to the correct pin

#endif // CONFIG_H
