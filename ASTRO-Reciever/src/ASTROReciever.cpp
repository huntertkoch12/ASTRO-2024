#include <SPI.h>
#include <RH_RF95.h>
#include <U8g2lib.h> // Include U8g2 library for the OLED
#include <WiFi.h>
#include <WebServer.h>
#include <TinyGPS++.h>

const char *ssid = "ESP32-Access-Point";
const char *password = "123456789";

TinyGPSPlus gps;

WebServer server(80); // HTTP server on port 80

String gpsData = "Waiting for GPS data..."; // Variable to store the latest GPS data
String onboardGPSData = "Waiting for onboard GPS data...";

void handleRoot();
void handleGPSData();

// T-Beam Pin Definitions
#define RADIO_SCLK_PIN 5
#define RADIO_MISO_PIN 19
#define RADIO_MOSI_PIN 27
#define RADIO_CS_PIN 18
#define RADIO_DIO0_PIN 26
#define RADIO_RST_PIN 23
#define RADIO_DIO1_PIN 33
#define RADIO_BUSY_PIN 32
#define BUTTON_PIN 38 // Button pin

// Frequency
#define RF95_FREQ 921.0

// Define Ejection
#define TX_POWER 23         // Transmit Power (LoRa, dBm)
#define BANDWIDTH 125       // Bandwidth (LoRa, kHz)
#define SPREADING_FACTOR 9  // Spreading Factor (LoRa), reduced from 12 for higher data rate
#define CODING_RATE 5       // Coding Rate (LoRa, 4/5), balance between error correction and data rate
#define PREAMBLE_LENGTH 8   // Preamble Length (LoRa), reduced from 12 for shorter time on air

// #define BANDWIDTH 125       // Bandwidth (LoRa, kHz)
// #define SPREADING_FACTOR 12 // Spreading Factor (LoRa)
// #define CODING_RATE 8       // Coding Rate (LoRa, 4/x)
// #define PREAMBLE_LENGTH 12  // Preamble Length (LoRa)

// OLED Pin Definitions
#define I2C_SDA 21
#define I2C_SCL 22

    // Singleton instance of the radio driver
    RH_RF95 rf95(RADIO_CS_PIN, RADIO_DIO0_PIN);

// OLED Display instance
U8G2_SSD1306_128X64_NONAME_F_SW_I2C u8g2(U8G2_R0, I2C_SCL, I2C_SDA, U8X8_PIN_NONE);

void displayMessage(const char *message);

void setup()
{
  pinMode(RADIO_RST_PIN, OUTPUT);
  digitalWrite(RADIO_RST_PIN, HIGH);

  Serial.begin(115200);
  Serial.println("LoRa Receiver");

  // Manual reset
  digitalWrite(RADIO_RST_PIN, LOW);
  delay(10);
  digitalWrite(RADIO_RST_PIN, HIGH);
  delay(10);

  if (!rf95.init())
  {
    Serial.println("LoRa radio init failed");
    while (1)
      ;
  }
  Serial.println("LoRa radio init OK!");

  if (!rf95.setFrequency(RF95_FREQ))
  {
    Serial.println("setFrequency failed");
    while (1)
      ;
  }

  Serial1.begin(9600, SERIAL_8N1, 34, 12); // RX, TX pins for T-Beam GPS

  Serial.print("Set Freq to: ");
  Serial.println(RF95_FREQ);

  rf95.setTxPower(TX_POWER, false);
  rf95.setSignalBandwidth(BANDWIDTH * 1000); // Convert kHz to Hz
  rf95.setSpreadingFactor(SPREADING_FACTOR);
  rf95.setCodingRate4(CODING_RATE);
  rf95.setPreambleLength(PREAMBLE_LENGTH);
  rf95.setPromiscuous(true);

  // Initialize the OLED display
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr); // Choose a suitable font
  displayMessage("Ready to receive"); // Initial display message

  // Set up WiFi AP
  WiFi.softAP(ssid, password);
  Serial.println("Access Point Started");
  Serial.print("IP Address: ");
  Serial.println(WiFi.softAPIP());

  // Define endpoints
  server.on("/", HTTP_GET, handleRoot);            // Serve the HTML page
  server.on("/gps-data", HTTP_GET, handleGPSData); // Endpoint to get GPS data

  server.begin(); // Start the server
}

void handleRoot()
{
  // HTML content with AJAX for updating GPS data
  String html = R"(
  <!DOCTYPE html>
  <html>
  <head><title>GPS Data</title></head>
  <body>
    <h1>GPS Data</h1>
    <div id="gps">Loading...</div>
    <script>
      function updateGPS() {
        fetch('/gps-data')
          .then(response => response.text())
          .then(data => {
            document.getElementById('gps').innerText = data;
          })
          .catch(console.error);
      }
      setInterval(updateGPS, 1000); // Update every second
    </script>
  </body>
  </html>
  )";
  server.send(200, "text/html", html);
}

void handleGPSData()
{
  String data = "Received GPS Data: " + gpsData + "\n" + onboardGPSData;
  server.send(200, "text/plain", data);
}

void loop()
{
  uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
  uint8_t len = sizeof(buf);

  if (rf95.waitAvailableTimeout(3000))
  { // Wait for a message
    if (rf95.recv(buf, &len))
    {                              // Check if a message is received
      buf[len] = 0;                // Ensure null termination
      Serial.println((char *)buf); // Print received message to Serial Monitor
      displayMessage((char *)buf); // Display received message on OLED

      // Update the global gpsData variable with the received data
      gpsData = String((char *)buf);

      // Attempt to print out RSSI and SNR
      float snr = rf95.lastSNR();
      Serial.print(F("SNR: "));
      Serial.println(snr);

      float rssi = rf95.lastRssi();
      Serial.print(F("RSSI: "));
      Serial.print(rssi);
      Serial.println(F(" dBm"));
    }
    else
    {
      Serial.println("Receive failed");
    }
  }

  while (Serial1.available() > 0)
  {
    if (gps.encode(Serial1.read()))
    {
      if (gps.location.isValid())
      {
        // Update the onboardGPSData string with the current coordinates
        onboardGPSData = "Onboard GPS - Lat: " + String(gps.location.lat(), 6) +
                         ", Lon: " + String(gps.location.lng(), 6) +
                         ", Alt: " + String(gps.altitude.meters()) +
                         ", Time: " + String(gps.time.hour()) + ":" +
                         String(gps.time.minute()) + ":" + String(gps.time.second());
      }
    }
  }

  // Handle any web server requests
  server.handleClient();
}

void displayMessage(const char *message)
{
  u8g2.clearBuffer();
  int textWidth = u8g2.getStrWidth(message);
  int x = (128 - textWidth) / 2; // Center horizontally
  int y = 32;                    // Center vertically
  u8g2.setCursor(x, y);
  u8g2.print(message);
  u8g2.sendBuffer();
}
