//*******************************************//
//             Include Libraries             //
//*******************************************//

#include <Arduino.h>
#include <RH_RF95.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_NeoPixel.h>

//*******************************************//
//             LoRa Settings                 //
//*******************************************//

#define RF95_FREQ 915.0            // LoRa Frequency (MHz)
#define RFM95_CS   16
#define RFM95_INT  21
#define RFM95_RST  17
#define TX_POWER 23                // Transmit Power (LoRa, dBm)
#define BANDWIDTH 125              // Bandwidth (LoRa, kHz)
#define SPREADING_FACTOR 12         // Spreading Factor (LoRa)
#define CODING_RATE 8              // Coding Rate (LoRa, 4/x)
#define PREAMBLE_LENGTH 12          // Preamble Length (LoRa)

//*******************************************//
//           External Declarations           //
//*******************************************//

extern RH_RF95 rf95;
extern Adafruit_NeoPixel strip;

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
