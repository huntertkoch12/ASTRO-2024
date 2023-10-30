// Include necessary libraries and configuration settings
#include "config.h"

// Create an instance of the Adafruit_BNO055 class to communicate with the BNO055 sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

// Initialize an instance of the RH_RF95 class for LoRa communication
RH_RF95 rf95(RFM95_CS, RFM95_INT);

// Function to initialize the LoRa radio
void initRadio() {
  // Configure the LED and Reset pins as output
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);

  // Initialize serial communication at a baud rate of 115200
  Serial.begin(115200);
  while (!Serial) {
    // Wait for the serial port to connect (necessary for Leonardo-type boards)
    delay(1);
  }
  delay(100);

  // Display a startup message
  Serial.println("Feather LoRa RX Test!");

  // Manually reset the LoRa radio
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);

  // Attempt to initialize the LoRa radio
  if (!rf95.init()) {
    Serial.println("LoRa radio initialization failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);  // Enter an infinite loop to halt further execution
  }
  Serial.println("LoRa radio initialization successful!");

  // Set the LoRa radio's operating frequency
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("Failed to set LoRa frequency!");
    while (1);  // Enter an infinite loop to halt further execution
  }
  Serial.print("Frequency set to: ");
  Serial.println(RF95_FREQ);

  // Configure the LoRa radio settings based on parameters defined in config.h
  rf95.setTxPower(TX_POWER, false);
  rf95.setSignalBandwidth(BANDWIDTH * 1000);
  rf95.setSpreadingFactor(SPREADING_FACTOR);
  rf95.setCodingRate4(CODING_RATE);
  rf95.setPreambleLength(PREAMBLE_LENGTH);
}

// Function to receive a message and send a reply
void receiveAndReply() {
  // Check if a message is available to be received
  if (rf95.available()) {
    // Buffer to store the incoming message
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    // Try to receive the message
    if (rf95.recv(buf, &len)) {
      // Indicate reception with the LED
      digitalWrite(LED_BUILTIN, HIGH);

      // Print the received message to the Serial Monitor
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Received message: ");
      Serial.println((char*)buf);
      Serial.print("Received Signal Strength Indicator (RSSI): ");
      Serial.println(rf95.lastRssi(), DEC);

      // Print the Signal to Noise Ratio (SNR) of the last received message
      Serial.print("Signal to Noise Ratio (SNR): ");
      Serial.println(rf95.lastSNR(), DEC);

      // Prepare the reply message
      uint8_t data[] = "And hello back to you!";

      // Send the reply
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Reply sent!");

      // Turn off the LED
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      // Inform if message reception failed
      Serial.println("Message reception failed!");
    }
  }
}

// Function to initialize the BNO055 sensor
void initBNO055() {
  // Initialize serial communication at a baud rate of 115200
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test");
  Serial.println("");

  // Attempt to initialize the BNO055 sensor
  if (!bno.begin()) {
    Serial.println("Oops, no BNO055 detected. Please check your wiring or I2C address!");
    while (1);  // Enter an infinite loop to halt further execution
  }
  delay(1000);

  // Use the external crystal for better accuracy
  bno.setExtCrystalUse(true);
}

// Function to log data from the BNO055 sensor
void logBNO055Data() {
  // Get the system status values (mainly for debugging purposes)
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  // Print out the system status values
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test: 0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error: 0x");
  Serial.println(system_error, HEX);
  
  // Get orientation data
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print("Orientation: Pitch ");
  Serial.print(euler.x());
  Serial.print(", Roll ");
  Serial.print(euler.y());
  Serial.print(", Yaw ");
  Serial.println(euler.z());

  // Get accelerometer data
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Serial.print("Accelerometer: X ");
  Serial.print(accel.x());
  Serial.print(", Y ");
  Serial.print(accel.y());
  Serial.print(", Z ");
  Serial.println(accel.z());

  // Get gyroscope data
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  Serial.print("Gyroscope: X ");
  Serial.print(gyro.x());
  Serial.print(", Y ");
  Serial.print(gyro.y());
  Serial.print(", Z ");
  Serial.println(gyro.z());

  // Get magnetometer data
  imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  Serial.print("Magnetometer: X ");
  Serial.print(mag.x());
  Serial.print(", Y ");
  Serial.print(mag.y());
  Serial.print(", Z ");
  Serial.println(mag.z());

  Serial.println("");
  delay(500);
}

