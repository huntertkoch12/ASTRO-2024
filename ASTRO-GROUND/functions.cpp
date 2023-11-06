//*******************************************//
//             Include Libraries             //
//*******************************************//

#include "config.h"

//*******************************************//
//       Create Sensor & Radio Instances     //
//*******************************************//

RH_RF95 rf95(RFM95_CS, RFM95_INT);
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

//*******************************************//
//            Initialization Block           //
//*******************************************//

// Initialize LoRa Radio
void initRadio() {

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  Serial.begin(115200);
  while (!Serial) delay(1);
  delay(100);
  
  Serial.println("LoRa Transmission Test!");
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  
  if (!rf95.init()) {
    Serial.println("LoRa initialization failed!");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info!");
    while (1) {
      strip.setPixelColor(0, strip.Color(255, 0, 0)); // Flash red to indicate error
      strip.show();
      delay(250);
      strip.clear();
      delay(250);
    }
  }

  Serial.println("LoRa initialization successful!");
  
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("Failed to set LoRa frequency!");
    while (1) {
      strip.setPixelColor(0, strip.Color(255, 0, 0)); // Flash red to indicate error
      strip.show();
      delay(250);
      strip.clear();
      delay(250);
    }
  }
  Serial.print("Frequency set to: ");
  Serial.println(RF95_FREQ);

  // Configure the LoRa radio settings based on parameters defined in config.h
  // Defaults after init are 434.0MHz, 13dBm, Bw = 125 kHz, Cr = 4/5, Sf = 128chips/symbol, CRC on
  rf95.setTxPower(TX_POWER, false);
  rf95.setSignalBandwidth(BANDWIDTH * 1000);
  rf95.setSpreadingFactor(SPREADING_FACTOR);
  rf95.setCodingRate4(CODING_RATE);
  rf95.setPreambleLength(PREAMBLE_LENGTH);
}