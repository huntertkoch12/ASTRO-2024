#include "config.h"

extern void initRadio();
extern void receiveAndReply();

void setup() {
  // Initialization for core 0 (if any)
  Serial.begin(115200);
  
  while (!Serial) delay(1);

  Serial.println("Initialization on Core 0!");
}

void loop() {
  // Main loop for core 0 (if any)
  // This can be left empty if not needed
}

void setup1() {
  initRadio();
}

void loop1() {
  receiveAndReply();
}

