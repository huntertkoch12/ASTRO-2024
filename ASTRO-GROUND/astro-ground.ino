#include "config.h"

//*******************************************//
//           External Declarations           //
//*******************************************//

// LoRa Radio Functions
extern void initRadio();

//*******************************************//
//                 Colors                    //
//*******************************************//

// Define a list of colors (R, G, B)
const uint32_t colorsSetup[] = {
  strip.Color(255, 215, 0), // Gold
  strip.Color(0, 255, 0),   // Green
  strip.Color(0, 0, 255),   // Blue
  strip.Color(255, 255, 0), // Yellow
  strip.Color(255, 0, 255), // Magenta
  strip.Color(0, 255, 255), // Cyan
  /* 
  strip.Color(255, 165, 0), // Orange
  strip.Color(255, 255, 255), // White
  strip.Color(128, 0, 128), // Purple
  strip.Color(255, 192, 203), // Pink
  strip.Color(128, 128, 128), // Grey
  strip.Color(128, 0, 0), // Maroon
  strip.Color(0, 128, 0), // Dark Green
  strip.Color(0, 0, 128), // Navy
  strip.Color(128, 128, 0), // Olive
  strip.Color(139, 69, 19), // Saddle Brown
  strip.Color(255, 69, 0), // Red-Orange
  strip.Color(255, 215, 0), // Gold
  strip.Color(124, 252, 0), // Lawn Green
  strip.Color(0, 250, 154), // Medium Spring Green
  strip.Color(72, 209, 204), // Medium Turquoise
  strip.Color(70, 130, 180), // Steel Blue
  strip.Color(123, 104, 238), // Medium Slate Blue
  strip.Color(255, 105, 180), // Hot Pink
  strip.Color(255, 20, 147), // Deep Pink
  strip.Color(0, 191, 255), // Deep Sky Blue
  strip.Color(218, 112, 214), // Orchid
  strip.Color(244, 164, 96), // Sandy Brown
  strip.Color(218, 165, 32), // Golden Rod
  strip.Color(240, 128, 128) // Light Coral
  */
};

const uint32_t colorsRuntime[] = {
  /*
  strip.Color(255, 0, 0),   // Red
  strip.Color(0, 255, 0),   // Green
  strip.Color(0, 0, 255),   // Blue
  strip.Color(255, 255, 0), // Yellow
  strip.Color(255, 0, 255), // Magenta
  strip.Color(0, 255, 255), // Cyan
  */ 
  strip.Color(255, 165, 0), // Orange
  strip.Color(255, 255, 255), // White
  strip.Color(128, 0, 128), // Purple
  strip.Color(255, 192, 203), // Pink
  /*
  strip.Color(128, 128, 128), // Grey
  strip.Color(128, 0, 0), // Maroon
  strip.Color(0, 128, 0), // Dark Green
  strip.Color(0, 0, 128), // Navy
  strip.Color(128, 128, 0), // Olive
  strip.Color(139, 69, 19), // Saddle Brown
  strip.Color(255, 69, 0), // Red-Orange
  strip.Color(255, 215, 0), // Gold
  strip.Color(124, 252, 0), // Lawn Green
  strip.Color(0, 250, 154), // Medium Spring Green
  strip.Color(72, 209, 204), // Medium Turquoise
  strip.Color(70, 130, 180), // Steel Blue
  strip.Color(123, 104, 238), // Medium Slate Blue
  strip.Color(255, 105, 180), // Hot Pink
  strip.Color(255, 20, 147), // Deep Pink
  strip.Color(0, 191, 255), // Deep Sky Blue
  strip.Color(218, 112, 214), // Orchid
  strip.Color(244, 164, 96), // Sandy Brown
  strip.Color(218, 165, 32), // Golden Rod
  strip.Color(240, 128, 128) // Light Coral
  */
};

// Variable to keep track of the current color index
int currentColorSetupIndex = 0;
int currentColorRunTimeIndex = 0;


// Color Function
const int numColorsSetup = sizeof(colorsSetup) / sizeof(colorsSetup[0]);
const int numColorRuntime = sizeof(colorsRuntime) / sizeof(colorsRuntime[0]);


//*******************************************//
//                 Setup                     //
//*******************************************//

void setup() {

    Serial.println("Initialization!");

    // Turn off BUILTIN LED
    digitalWrite(LED_BUILTIN, LOW);


    // Initialize LED for debugging
    strip.begin();
    strip.setBrightness(255);  // Set brightness (0-255)
    strip.show(); // Initialize all pixels to 'off'
    changeColorSetup();
    delay(500); // Optional: Add a delay to make the color change noticeable
    
    // Initialize I2C and SPI
    Wire.begin();  // Use default I2C pins
    SPI.begin();   // Use default SPI pins
    changeColorSetup();
    delay(500); // Optional: Add a delay to make the color change noticeable  
}



//*******************************************//
//                 Loop                      //
//*******************************************//

void loop() {
  if (rf95.available()) {
    // Should be a message for us now
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      digitalWrite(LED_BUILTIN, HIGH);
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Got: ");
      Serial.println((char*)buf);
       Serial.print("RSSI: ");
      Serial.println(rf95.lastRssi(), DEC);

    } else {
      Serial.println("Receive failed");
    }

    digitalWrite(LED_BUILTIN, LOW);
  }
}

//*******************************************//
//             Color Function                //
//*******************************************//

void changeColorSetup() {
  // Set the LED to the current color
  strip.setPixelColor(0, colorsSetup[currentColorSetupIndex]);
  strip.show();

  // Update the color index for the next loop iteration
  currentColorSetupIndex = (currentColorSetupIndex + 1) % numColorsSetup;
}

void changeColorRuntime() {
  // Set the LED to the current color
  strip.setPixelColor(0, colorsRuntime[currentColorRunTimeIndex]);
  strip.show();

  // Update the color index for the next loop iteration
  currentColorRunTimeIndex = (currentColorRunTimeIndex + 1) % numColorRuntime;
}
