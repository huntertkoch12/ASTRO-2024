/*
_____/\\\\\\\\\________/\\\\\\\\\\\____/\\\\\\\\\\\\\\\____/\\\\\\\\\___________/\\\\\______
 ___/\\\\\\\\\\\\\____/\\\/////////\\\_\///////\\\/////___/\\\///////\\\_______/\\\///\\\____
  __/\\\/////////\\\__\//\\\______\///________\/\\\_______\/\\\_____\/\\\_____/\\\/__\///\\\__
   _\/\\\_______\/\\\___\////\\\_______________\/\\\_______\/\\\\\\\\\\\/_____/\\\______\//\\\_
    _\/\\\\\\\\\\\\\\\______\////\\\____________\/\\\_______\/\\\//////\\\____\/\\\_______\/\\\_
     _\/\\\/////////\\\_________\////\\\_________\/\\\_______\/\\\____\//\\\___\//\\\______/\\\__
      _\/\\\_______\/\\\__/\\\______\//\\\________\/\\\_______\/\\\_____\//\\\___\///\\\__/\\\____
       _\/\\\_______\/\\\_\///\\\\\\\\\\\/_________\/\\\_______\/\\\______\//\\\____\///\\\\\/_____
        _\///________\///____\///////////___________\///________\///________\///_______\/////_______

     /___.`--.____ .--. ____.--(
                       .'_.- (    ) -._'.
                     .'.'    |'..'|    '.'.
              .-.  .' /'--.__|____|__.--'\ '.  .-.
             (O).)-| |  \    |    |    /  | |-(.(O)
              `-'  '-'-._'-./      \.-'_.-'-'  `-'
                 _ | |   '-.________.-'   | | _
              .' _ | |     |   __   |     | | _ '.
             / .' ''.|     | /    \ |     |.'' '. \
             | |( )| '.    ||      ||    .' |( )| |
             \ '._.'   '.  | \    / |  .'   '._.' /
              '.__ ______'.|__'--'__|.'______ __.'
             .'_.-|         |------|         |-._'.
            //\\  |         |--::--|         |  //\\
           //  \\ |         |--::--|         | //  \\
          //    \\|        /|--::--|\        |//    \\
         / '._.-'/|_______/ |--::--| \_______|\`-._.' \
        / __..--'        /__|--::--|__\        `--..__ \
       / /               '-.|--::--|.-'               \ \
      / /                   |--::--|                   \ \
     / /                    |--::--|                    \ \
 _.-'  `-._                 _..||.._                  _.-` '-._
'--..__..--'               '-.____.-'                '--..__..-'

*/

#include "config.h"

void changeColorSetup();
void changeColorRuntime();
void printRTCDateTime();

//*******************************************//
//           External Declarations           //
//*******************************************//

// LoRa Radio Functions
extern void initRadio();
extern void receiveAndReply();

// BNO055 Sensor Functions
extern void initBNO055();
extern void logBNO055Data();

// SD Card Functions
extern void logDataToSD();
extern void setupSDCard();

// Alitemter Functions
extern void initMPL3115A2();
extern void logMPL3115A2Data();

// GPS Functions
extern void initGPS();
extern void transmitGPSData();
extern void rawGPS();
extern void setGPSUpdateRate(int milliseconds);

// Backup IMU Function
extern void initLSM6DSO32();

// ENS160 Functions
extern void initENS160();
extern void logENS160();

// BME688
extern void initBME688();
extern void logBME688();

// Microphone
extern void initMicrophone();
extern void logMicrophone();

// RTC Functions
extern void setupRTC();

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
    strip.Color(255, 165, 0),   // Orange
    strip.Color(255, 255, 255), // White
    strip.Color(128, 0, 128),   // Purple
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

void setup()
{

  Wire.begin();

  Serial.println("Initialization on Core 0!");

  // Turn off BUILTIN LED
  digitalWrite(LED_BUILTIN, LOW);

  // Initialize LED for debugging
  strip.begin();
  strip.setBrightness(255); // Set brightness (0-255)
  strip.show();             // Initialize all pixels to 'off'
  changeColorSetup();
  delay(500); // Optional: Add a delay to make the color change noticeable

  // Initialize I2C and SPI
  Wire.begin(); // Use default I2C pins
  SPI.begin();  // Use default SPI pins
  changeColorSetup();
  delay(500);

  // // Initialize Sensor and SD Card
  initBNO055();
  changeColorSetup();
  delay(500);

  setupSDCard();
  changeColorSetup();
  delay(500);

  initMPL3115A2();
  changeColorSetup();
  delay(500);

  // initGPS();
  // setGPSUpdateRate(2000);
  // changeColorSetup();
  // delay(500);

  initLSM6DSO32();
  changeColorSetup();
  delay(500);

  initENS160();
  changeColorSetup();
  delay(500);

  initBME688();
  changeColorSetup();
  delay(500);

  initMicrophone();
  changeColorSetup();
  delay(500);

  setupRTC();
  changeColorSetup();
  delay(500);
}

//*******************************************//
//                 Loop                      //
//*******************************************//

void loop()
{

  // Log Data to SD Card
  logDataToSD();

  /* Debug */

  // // Log Data from BNO055 Sensor
  // logBNO055Data();

  // Log Data from Altimeter Sensor
  // logMPL3115A2Data();

  // Log Data from ENS160 Sensor
  // logENS160();

  // Log Data from BME688 Sensor
  // logBME688();

  // Log Data from Microphone Sensor
  // logMicrophone();

  // Log Data from RTC Clock
  // printRTCDateTime();
}

//*******************************************//
//                Setup1                     //
//*******************************************//

void setup1()
{

  // Initialize LoRa Radio
  // initRadio();
}

//*******************************************//
//                Loop1                      //
//*******************************************//

void loop1()
{
  // Handle LoRa Communication
  // rawGPS();
  // transmitGPSData();
}

//*******************************************//
//             Color Function                //
//*******************************************//

void changeColorSetup()
{
  // Set the LED to the current color
  strip.setPixelColor(0, colorsSetup[currentColorSetupIndex]);
  strip.show();

  // Update the color index for the next loop iteration
  currentColorSetupIndex = (currentColorSetupIndex + 1) % numColorsSetup;
}

void changeColorRuntime()
{
  // Set the LED to the current color
  strip.setPixelColor(0, colorsRuntime[currentColorRunTimeIndex]);
  strip.show();

  // Update the color index for the next loop iteration
  currentColorRunTimeIndex = (currentColorRunTimeIndex + 1) % numColorRuntime;
}

// Print RTC for debugging:

void printRTCDateTime()
{
  DateTime now = rtc.now();
  char buf[] = "YYYY-MM-DD hh:mm:ss";
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  Serial.println(buf);
}
