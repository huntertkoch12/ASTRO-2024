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
String gpsSentence = ""; // Buffer to hold incoming GPS data

void handleRoot();
void handleGPSData();
void updateOnboardGPSData(String sentence);
double convertToDegrees(String nmeaPos);

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
#define TX_POWER 23        // Transmit Power (LoRa, dBm)
#define BANDWIDTH 125      // Bandwidth (LoRa, kHz)
#define SPREADING_FACTOR 9 // Spreading Factor (LoRa), reduced from 12 for higher data rate
#define CODING_RATE 5      // Coding Rate (LoRa, 4/5), balance between error correction and data rate
#define PREAMBLE_LENGTH 8  // Preamble Length (LoRa), reduced from 12 for shorter time on air

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

void loop()
{
  static unsigned long lastGPSTime = 0; // Last time the GPS data was updated
  unsigned long currentMillis = millis();

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

  // Read the GPS data
  while (Serial1.available() > 0)
  {
    char c = Serial1.read();

    // Print each character as it's read from the GPS
    Serial.print(c);

    // Append the read character to the gpsSentence buffer
    if (c != '\n' && c != '\r')
    { // Ignore newline and carriage return characters
      gpsSentence += c;
    }

    // Check if the sentence is complete (end with newline)
    if (c == '\n')
    {
      // Check if the sentence is a GPGGA sentence
      if (gpsSentence.startsWith("$GPGGA"))
      {
        updateOnboardGPSData(gpsSentence); // Update GPS data using the complete sentence
      }
      gpsSentence = ""; // Clear the buffer for the next sentence
    }
  }

  // Handle any web server requests
  server.handleClient();
}

// Functions

void updateOnboardGPSData(String sentence)
{
  // Split the sentence into its comma-separated components
  String fields[15]; // GPGGA has up to 15 fields
  int fieldIndex = 0;

  for (int i = 0; i < sentence.length() && fieldIndex < 15; i++)
  {
    int commaIndex = sentence.indexOf(',', i);
    if (commaIndex == -1)
    { // No more commas
      fields[fieldIndex++] = sentence.substring(i);
      break;
    }
    else
    {
      fields[fieldIndex++] = sentence.substring(i, commaIndex);
      i = commaIndex;
    }
  }

  // Now fields[] contains all the parts of the GPGGA sentence
  String time = fields[1];
  String latitude = fields[2];
  String latDirection = fields[3];
  String longitude = fields[4];
  String lonDirection = fields[5];
  String fixQuality = fields[6];
  String satellites = fields[7];
  String altitude = fields[9];

  // Convert latitude and longitude to a more readable format
  double lat = convertToDegrees(latitude);
  if (latDirection == "S")
    lat = -lat;
  double lon = convertToDegrees(longitude);
  if (lonDirection == "W")
    lon = -lon;

  // Update the onboardGPSData with the parsed values
  onboardGPSData = "Onboard GPS - Lat: " + String(lat, 6) +
                   ", Lon: " + String(lon, 6) +
                   ", Alt: " + altitude +
                   ", Time: " + time.substring(0, 2) + ":" +
                   time.substring(2, 4) + ":" + time.substring(4, 6);
  Serial.println(onboardGPSData); // Print the GPS data string for debugging
}

double convertToDegrees(String nmeaPos)
{
  double rawValue = nmeaPos.toDouble();
  int degrees = int(rawValue / 100);
  double minutes = rawValue - (degrees * 100);
  return degrees + (minutes / 60);
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

void handleRoot()
{
  String html = R"html(
  <!DOCTYPE html>
  <html>
  <head>
    <title>GPS Data</title>
    <style>
      #arrow {
        width: 0; 
        height: 0; 
        border-left: 20px solid transparent;
        border-right: 20px solid transparent;
        border-bottom: 40px solid blue;
        margin: 20px;
        transform-origin: 50% 20%;
        transition: transform 0.5s ease-out;
      }
      button {
        padding: 10px 20px;
        font-size: 16px;
        cursor: pointer;
      }
    </style>
  </head>
  <body>
    <h1>GPS Data</h1>
    <div id="gps">Loading...</div>
    <div id="arrow"></div>
    <button onclick="activateSpeech()">Activate Speech</button>
    <script>
      let speechActivated = false;
      let currentAltitude = '';

      function activateSpeech() {
        speechActivated = true;
        speak("Speech activated.");
      }

      function speak(text) {
        if (!speechActivated) return; // Only speak if activated by the user

        // Cancel the queue if it's too long to avoid delays
        if (speechSynthesis.speaking || speechSynthesis.pending) {
          speechSynthesis.cancel();
        }

        const utterance = new SpeechSynthesisUtterance(text);
        speechSynthesis.speak(utterance);
      }

      function updateGPS() {
        fetch('/gps-data')
          .then(response => response.text())
          .then(data => {
            document.getElementById('gps').innerText = data;
            const lines = data.split('\\n');
            const bearingLine = lines.find(line => line.startsWith('Bearing:'));
            if (bearingLine) {
              const bearing = parseFloat(bearingLine.split(':')[1]);
              document.getElementById('arrow').style.transform = 'rotate(' + bearing + 'deg)';
            }
            const altitudeLine = lines.find(line => line.includes('Alt:'));
            if (altitudeLine) {
              // Truncate the altitude to a whole number
              const newAltitude = Math.floor(parseFloat(altitudeLine.split('Alt: ')[1].split(' ')[0]));
              if (newAltitude !== currentAltitude) {
                currentAltitude = newAltitude;
                // Speak the new altitude if speech is activated
                if (speechActivated) {
                  speak(currentAltitude + ' meters');
                }
              }
            }
          })
          .catch(console.error);
      }

      setInterval(updateGPS, 1000); // Update every second
    </script>
  </body>
  </html>
  )html";
  server.send(200, "text/html", html);
}

float calcBearing(float lat1, float long1, float lat2, float long2)
{
  lat1 = radians(lat1);
  long1 = radians(long1);
  lat2 = radians(lat2);
  long2 = radians(long2);

  float y = sin(long2 - long1) * cos(lat2);
  float x = cos(lat1) * sin(lat2) - sin(lat1) * cos(lat2) * cos(long2 - long1);
  float bearing = atan2(y, x);
  bearing = degrees(bearing);
  bearing = fmod((bearing + 360.0), 360.0); // Normalize to 0-360
  return bearing;
}

void handleGPSData() {
  // Parse target latitude and longitude from the gpsData string
  int latStart = gpsData.indexOf("Lat: ") + 5;
  int latEnd = gpsData.indexOf(",", latStart);
  float targetLat = gpsData.substring(latStart, latEnd).toFloat();

  int lonStart = gpsData.indexOf("Lon: ", latEnd) + 5;
  int lonEnd = gpsData.indexOf(",", lonStart);
  float targetLong = gpsData.substring(lonStart, lonEnd).toFloat();

  // Parse current latitude and longitude from the onboardGPSData string
  latStart = onboardGPSData.indexOf("Lat: ") + 5;
  latEnd = onboardGPSData.indexOf(",", latStart);
  float currentLat = onboardGPSData.substring(latStart, latEnd).toFloat();

  lonStart = onboardGPSData.indexOf("Lon: ", latEnd) + 5;
  lonEnd = onboardGPSData.indexOf(",", lonStart);
  float currentLong = onboardGPSData.substring(lonStart, lonEnd).toFloat();

  // Calculate the bearing from current location to target location
  float bearing = calcBearing(currentLat, currentLong, targetLat, targetLong);

  // Prepare the data string to send back to the client
  String data = "Received GPS Data: " + gpsData + "\n" +
                "Onboard GPS Data: " + onboardGPSData + "\n" +
                "Bearing: " + String(bearing, 2) + "°";

  // Send the data as a plain text response
  server.send(200, "text/plain", data);
}

// void handleRoot()
    // {
    //   // HTML content with AJAX for updating GPS data
    //   String html = R"(
    //   <!DOCTYPE html>
    //   <html>
    //   <head><title>GPS Data</title></head>
    //   <body>
    //     <h1>GPS Data</h1>
    //     <div id="gps">Loading...</div>
    //     <script>
    //       function updateGPS() {
    //         fetch('/gps-data')
    //           .then(response => response.text())
    //           .then(data => {
    //             document.getElementById('gps').innerText = data;
    //           })
    //           .catch(console.error);
    //       }
    //       setInterval(updateGPS, 1000); // Update every second
    //     </script>
    //   </body>
    //   </html>
    //   )";
    //   server.send(200, "text/html", html);
    // }
