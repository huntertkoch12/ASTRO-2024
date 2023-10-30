//*******************************************//
//             Include Libraries             //
//*******************************************//

#include "config.h"

//*******************************************//
//       Create Sensor & Radio Instances     //
//*******************************************//

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
RH_RF95 rf95(RFM95_CS, RFM95_INT);
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();

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
  
  Serial.println("Feather LoRa RX Test!");
  digitalWrite(RFM95_RST, LOW);
  delay(10);
  digitalWrite(RFM95_RST, HIGH);
  delay(10);
  
  if (!rf95.init()) {
    Serial.println("LoRa radio initialization failed");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info");
    while (1);
  }
  Serial.println("LoRa radio initialization successful!");
  
  if (!rf95.setFrequency(RF95_FREQ)) {
    Serial.println("Failed to set LoRa frequency!");
    while (1);
  }
  Serial.print("Frequency set to: ");
  Serial.println(RF95_FREQ);
  
  rf95.setTxPower(TX_POWER, false);
  rf95.setSignalBandwidth(BANDWIDTH * 1000);
  rf95.setSpreadingFactor(SPREADING_FACTOR);
  rf95.setCodingRate4(CODING_RATE);
  rf95.setPreambleLength(PREAMBLE_LENGTH);
}

// Initialize BNO055 Sensor
void initBNO055() {
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test\n");
  
  if (!bno.begin()) {
    Serial.println("Oops, no BNO055 detected. Please check your wiring or I2C address!");
    while (1);
  }
  delay(1000);
  bno.setExtCrystalUse(true);
}

// Initialize SD Card
const int chipSelect = 10;
SdFat sd;
SdFile dataFile;

void setupSDCard() {
  Serial.print("Initializing SD card...");
  
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  
  if (dataFile.open("datalog.txt", O_WRITE | O_CREAT | O_APPEND)) {
  dataFile.println("Time, Pitch, Roll, Yaw, AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ, Pressure, Altitude, Temperature");
  dataFile.close();
  Serial.println("Data log started");
} else {
  Serial.println("error opening datalog.txt");
}

}

// Initalize Altimeter

void initMPL3115A2() {
  Serial.println("Initializing MPL3115A2 Sensor");
  
  if (!baro.begin()) {
    Serial.println("Could not find a valid MPL3115A2 sensor, check wiring!");
    while (1);
  }
}


//*******************************************//
//              Runtime Block                //
//*******************************************//

// Receive Message and Send Reply
void receiveAndReply() {
  if (rf95.available()) {
    uint8_t buf[RH_RF95_MAX_MESSAGE_LEN];
    uint8_t len = sizeof(buf);

    if (rf95.recv(buf, &len)) {
      digitalWrite(LED_BUILTIN, HIGH);
      RH_RF95::printBuffer("Received: ", buf, len);
      Serial.print("Received message: ");
      Serial.println((char*)buf);
      Serial.print("Received Signal Strength Indicator (RSSI): ");
      Serial.println(rf95.lastRssi(), DEC);
      Serial.print("Signal to Noise Ratio (SNR): ");
      Serial.println(rf95.lastSNR(), DEC);
      
      uint8_t data[] = "And hello back to you!";
      rf95.send(data, sizeof(data));
      rf95.waitPacketSent();
      Serial.println("Reply sent!");
      digitalWrite(LED_BUILTIN, LOW);
    } else {
      Serial.println("Message reception failed!");
    }
  }
}

// Log BNO055 Sensor Data
void logBNO055Data() {
  uint8_t system_status, self_test_results, system_error;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test: 0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error: 0x");
  Serial.println(system_error, HEX);
  
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  Serial.print("Orientation: Pitch ");
  Serial.print(euler.x());
  Serial.print(", Roll ");
  Serial.print(euler.y());
  Serial.print(", Yaw ");
  Serial.println(euler.z());
  
  imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  Serial.print("Accelerometer: X ");
  Serial.print(accel.x());
  Serial.print(", Y ");
  Serial.print(accel.y());
  Serial.print(", Z ");
  Serial.println(accel.z());
  
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  Serial.print("Gyroscope: X ");
  Serial.print(gyro.x());
  Serial.print(", Y ");
  Serial.print(gyro.y());
  Serial.print(", Z ");
  Serial.println(gyro.z());
  
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

// Log Data to SD Card
void logDataToSD() {
  // Open the file
  if (!dataFile.open("datalog.txt", O_WRITE | O_CREAT | O_APPEND)) {
    Serial.println("Error opening datalog.txt");
    return;
  }

  // If the file is open, write to it
  if (dataFile) {
    // Get current time since the program started
    unsigned long currentTime = millis();

    // Get BNO055 data
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    // Get MPL3115A2 data
    float pressure = baro.getPressure();   // Pressure in Pascals
    float altitude = baro.getAltitude();   // Altitude in meters
    float temperature = baro.getTemperature(); // Temperature in degrees Celsius

    // Write data to SD card
    dataFile.print(currentTime);
    dataFile.print(", ");
    dataFile.print(euler.x()); dataFile.print(", "); dataFile.print(euler.y()); dataFile.print(", "); dataFile.print(euler.z());
    dataFile.print(", ");
    dataFile.print(accel.x()); dataFile.print(", "); dataFile.print(accel.y()); dataFile.print(", "); dataFile.print(accel.z());
    dataFile.print(", ");
    dataFile.print(gyro.x()); dataFile.print(", "); dataFile.print(gyro.y()); dataFile.print(", "); dataFile.print(gyro.z());
    dataFile.print(", ");
    dataFile.print(mag.x()); dataFile.print(", "); dataFile.print(mag.y()); dataFile.print(", "); dataFile.print(mag.z());
    dataFile.print(", ");
    dataFile.print(pressure); dataFile.print(", "); dataFile.print(altitude); dataFile.print(", "); dataFile.println(temperature);

    // Close the file
    dataFile.close();
  } else {
    // If the file didn't open, print an error
    Serial.println("Error opening datalog.txt");
  }
}


// Log MPL3115A2 Sensor Data to Serial Monitor
void logMPL3115A2Data() {
  float pressure = baro.getPressure();   // Pressure in Pascals
  float altitude = baro.getAltitude();   // Altitude in meters
  float temperature = baro.getTemperature(); // Temperature in degrees Celsius

  Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" Pa");
  Serial.print("Altitude: "); Serial.print(altitude); Serial.println(" m");
  Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" C");
}
