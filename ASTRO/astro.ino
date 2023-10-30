#include "config.h"

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

//*******************************************//
//                 Setup                     //
//*******************************************//

void setup() {
    // Initialize Serial Communication
    Serial.begin(115200);
    while (!Serial) {
        delay(1); // Wait for Serial Connection
    }
    Serial.println("Initialization on Core 0!");

    // Initialize Sensor and SD Card
    initBNO055();
    setupSDCard();
}

//*******************************************//
//                 Loop                      //
//*******************************************//

void loop() {
    // Log Data from BNO055 Sensor
    logBNO055Data();
    
    // Log Data to SD Card
    logDataToSD();
}

//*******************************************//
//                Setup1                     //
//*******************************************//

void setup1() {
    // Initialize LoRa Radio
    initRadio();
}

//*******************************************//
//                Loop1                      //
//*******************************************//

void loop1() {
    // Handle LoRa Communication
    receiveAndReply();
}
