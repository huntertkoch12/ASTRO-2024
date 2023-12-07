#include <SPI.h>
#include <RH_RF95.h>
#include <U8g2lib.h>  // Include U8g2 library for the OLED

// T-Beam Pin Definitions
#define RADIO_SCLK_PIN               5
#define RADIO_MISO_PIN              19
#define RADIO_MOSI_PIN              27
#define RADIO_CS_PIN                18
#define RADIO_DIO0_PIN              26
#define RADIO_RST_PIN               23
#define RADIO_DIO1_PIN              33
#define RADIO_BUSY_PIN              32
#define BUTTON_PIN                  38  // Button pin

// Frequency
#define RF95_FREQ 905.0

// Define Ejection
#define TX_POWER 23                // Transmit Power (LoRa, dBm)
#define BANDWIDTH 125              // Bandwidth (LoRa, kHz)
#define SPREADING_FACTOR 12        // Spreading Factor (LoRa)
#define CODING_RATE 8              // Coding Rate (LoRa, 4/x)
#define PREAMBLE_LENGTH 12         // Preamble Length (LoRa)

// OLED Pin Definitions
#define I2C_SDA                     21
#define I2C_SCL                     22

// Singleton instance of the radio driver
RH_RF95 rf95(RADIO_CS_PIN, RADIO_DIO0_PIN);

// OLED Display instance
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, I2C_SCL, I2C_SDA, U8X8_PIN_NONE);

void setup() {
  pinMode(RADIO_RST_PIN, OUTPUT);
  digitalWrite(RADIO_RST_PIN, HIGH);

  Serial.begin(115200);
  Serial.println("LoRa Receiver");

  // Manual reset
  digitalWrite(RADIO_RST_PIN, LOW);
  delay(10);
  digitalWrite(RADIO_RST_PIN, HIGH);
  delay(10);

  if (!rf95.init()) {
    Serial.println("LoRa radio init failed");
    while (1);
  }
  Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("setFrequency failed");
    while (1);
  }
  Serial.print("Set Freq to: "); Serial.println(RF95_FREQ);

  rf95.setTxPower(TX_POWER, false);
  rf95.setSignalBandwidth(BANDWIDTH * 1000); // Convert kHz to Hz
  rf95.setSpreadingFactor(SPREADING_FACTOR);
  rf95.setCodingRate4(CODING_RATE);
  rf95.setPreambleLength(PREAMBLE_LENGTH);

  // Initialize the OLED display
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr); // Choose a suitable font
  displayMessage("Ready to receive");  // Initial display message
}

void loop() {
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(3000)) {  // Wait for a message
    if (rf95.recv(buf, &len)) {  // Check if a message is received
      buf[len] = 0; // Ensure null termination
      Serial.println((char*)buf); // Print received message to Serial Monitor
      displayMessage((char*)buf); // Display received message on OLED
    } else {
      Serial.println("Receive failed");
    }
  }
}

void displayMessage(const char *message) {
  u8g2.clearBuffer();
  int textWidth = u8g2.getStrWidth(message);
  int x = (128 - textWidth) / 2; // Center horizontally
  int y = 32; // Center vertically
  u8g2.setCursor(x, y);
  u8g2.print(message);
  u8g2.sendBuffer();
}
