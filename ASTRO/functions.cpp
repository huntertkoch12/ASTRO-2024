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
Adafruit_GPS GPS(&Serial1);  // Assuming you are using UART1 for GPS
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
RTC_DS3231 rtc;
DFRobot_H3LIS200DL_I2C acce;
Adafruit_LSM6DSO32 lsm6dso32;

//*******************************************//
//            Initialization Block           //
//*******************************************//

// Initialize LoRa Radio
void initRadio() {

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(RFM95_RST, OUTPUT);
  digitalWrite(RFM95_RST, HIGH);
  
  Serial.begin(115200);
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

void bno_write(uint8_t i2c_addr, uint8_t reg, uint8_t data)  // Write to Register
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true);  // Send Stop
}

// Initialize BNO055 Sensor
void initBNO055() {
  Serial.begin(115200);
  Serial.println("Orientation Sensor Test\n");

  if (!bno.begin()) {
    Serial.println("Oops, no BNO055 detected. Please check your wiring or I2C address!");
    while (1) {
      strip.setPixelColor(0, strip.Color(255, 0, 0)); // Flash red to indicate error
      strip.show();
      delay(250);
      strip.clear();
      delay(250);
    }
  }

  // Set the BNO055 accelerometer to ±16g range
  bno_write(BNO_ADDR, PAGE_ID, 1);        // Switch to register page 1
  bno_write(BNO_ADDR, ACC_CONFIG, 0x0F);  // Set accelerometer range to ±16g
  bno_write(BNO_ADDR, PAGE_ID, 0);        // Switch back to register page 0

  // Additional configuration as needed


  delay(1000);
  bno.setExtCrystalUse(true);

  // Declare variables
  uint8_t system_status, self_test_results, system_error;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test: 0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error: 0x");
  Serial.println(system_error, HEX);
}


void initDF_Robot(){
  Serial.println("Initializing DF_Robot");
  while(!acce.begin()){
    Serial.println("DF_Robot initialization failed!");
    strip.setPixelColor(0, strip.Color(255, 0, 0)); // Flash red to indicate error
    strip.show();
    delay(250);
    strip.clear();
    delay(250);
  }
  acce.setRange(DFRobot_LIS::eH3lis200dl_100g);
  acce.setAcquireRate(DFRobot_LIS::eNormal_50HZ);

  delay(1000);
  Serial.println("DF_Robot Initialization Complete!");

}

void initLSM6DSO32() {
  Serial.println("LSM6DSO32 Sensor Test");

  if (!lsm6dso32.begin_I2C()) {
    Serial.println("Failed to find LSM6DSO32 chip");
    while (1) {
      strip.setPixelColor(0, strip.Color(255, 0, 0)); // Flash red to indicate error
      strip.show();
      delay(250);
      strip.clear();
      delay(250);
    }
  }

  Serial.println("LSM6DSO32 Found!");
  lsm6dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_32_G);
  lsm6dso32.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS);

}


void handleError() {
  while (1) {
    strip.setPixelColor(0, strip.Color(255, 0, 0)); // Flash red to indicate error
    strip.show();
    delay(250);
    strip.clear();
    delay(250);
  }
}


// Initialize SD Card
const int chipSelect = 10;
SdFat sd;
SdFile bno055File, lsm6dso32File, otherFile;

const char* BNO055_FILE_NAME = "BNO055Data.csv";
const char* LSM6DSO32_FILE_NAME = "LSM6DSO32Data.csv";
const char* OTHER_FILE_NAME = "OtherData.csv";

void setupSDCard() {
  Serial.print("Initializing SD card...");
  
  if (!sd.begin(chipSelect, SPI_FULL_SPEED)) {
    Serial.println("Initialization failed!");
    while (1) {
      strip.setPixelColor(0, strip.Color(255, 0, 0)); // Flash red to indicate error
      strip.show();
      delay(250);
      strip.clear();
      delay(250);
    }
  }
  Serial.println("Initialization done.");

  // Initialize BNO055 data file
  if (!bno055File.open(BNO055_FILE_NAME, O_WRITE | O_CREAT | O_APPEND)) {
    Serial.println("Error opening BNO055 data file!");
    handleError();
  }
  // Write headers for BNO055 file
  bno055File.println("Timestamp, AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ, Temp");
  bno055File.close();

  // Initialize LSM6DSO32 data file
  if (!lsm6dso32File.open(LSM6DSO32_FILE_NAME, O_WRITE | O_CREAT | O_APPEND)) {
    Serial.println("Error opening LSM6DSO32 data file!");
    handleError();
  }
  // Write headers for LSM6DSO32 file
  lsm6dso32File.println("Timestamp, AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ");
  lsm6dso32File.close();

  // Initialize Other data file
  if (!otherFile.open(OTHER_FILE_NAME, O_WRITE | O_CREAT | O_APPEND)) {
    Serial.println("Error opening Other data file!");
    handleError();
  }
  // Write headers or other initialization data for Other file
  otherFile.println("Timestamp, MPL_Pressure, MPL_Altitude, MPL_Temperature, DFLIS_AccelX, DFLIS_AccelY, DFLIS_AccelZ");
  otherFile.close();

  Serial.println("All data logs started.");
}

// Initalize Altimeter
void initMPL3115A2() {
  Serial.println("Initializing MPL3115A2 Sensor");
  
  if (!baro.begin()) {
    Serial.println("Could not find a valid MPL3115A2 sensor, check wiring!");
    while (1) {
      strip.setPixelColor(0, strip.Color(255, 0, 0)); // Flash red to indicate error
      strip.show();
      delay(250);
      strip.clear();
      delay(250);
    }
  }

  // Set a different oversampling rate
  baro.setOversamplingRate(MPL3115A2_CTRL_REG1_OS128); // Example: Set to OS4
  
  Serial.println("MPL3115A2 Sensor initialized with custom sampling rate.");
}

// Initialize the GPS
void initGPS() {
  GPS.begin(9600);
  // Flush any old data from the GPS serial buffer
  while (GPS.available() > 0) {
    GPS.read();
  }
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
}

// Initalize RTC
void setupRTC() {

  if (!rtc.begin()) {
    Serial.println("Couldn't find RTC!");
    while (1) {
      strip.setPixelColor(0, strip.Color(255, 0, 0)); // Flash red to indicate error
      strip.show();
      delay(250);
      strip.clear();
      delay(250);
    }
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, please set the time!");
    // Manually set the date and time
    // rtc.adjust(DateTime(2023, 1, 21, 3, 0, 0));
    // Or use the compile time: (this will not be accurate if the RTC has lost power)
    //rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

// Set GPS rate
void setGPSUpdateRate(int milliseconds) {
  // Create a buffer for the command string
  char command[20];
  
  // Format the command string with the desired update interval
  snprintf(command, sizeof(command), "$PMTK220,%d*1C\r\n", milliseconds);
  
  // Send the command to the GPS module
  GPS.print(command);
  
  // Wait for the command to be sent
  delay(100);
}
//*******************************************//
//              Runtime Block                //
//*******************************************//

// Get Time
String getTimeStamp() {
  DateTime now = rtc.now();
  char buf[] = "YYYY-MM-DD hh:mm:ss";
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  return String(buf);
}

void transmitGPSData() {
  static unsigned int packetCounter = 0; // Packet counter

  Serial.println("Checking GPS Data..."); // Debug print

  // Check if new NMEA data is available
  while (GPS.available()) {
    char c = GPS.read();
    Serial.print(c); // Print raw GPS data
    if (GPS.newNMEAreceived()) {
      Serial.println("New NMEA sentence received."); // Debug print
      if (!GPS.parse(GPS.lastNMEA())) {
        Serial.println("Failed to parse NMEA sentence."); // Debug print
        break; // Exit the loop if the sentence can't be parsed
      }
      
      // Check if we have a GPS fix
      if (GPS.fix) {
        Serial.println("GPS fix obtained."); // Debug print
        packetCounter++; // Increment the packet counter

        // Prepare the GPS data into a transmission buffer
        char transmitBuffer[60]; // Increased buffer size to accommodate packet counter
        snprintf(transmitBuffer, sizeof(transmitBuffer), "Packet %u, Lat:%f%s, Lon:%f%s",
                 packetCounter,
                 GPS.latitude, GPS.lat == 'N' ? "N" : "S",
                 GPS.longitude, GPS.lon == 'E' ? "E" : "W");

        // Transmit the GPS data
        rf95.send((uint8_t *)transmitBuffer, strlen(transmitBuffer) + 1); // +1 to include null-terminator
        Serial.println(transmitBuffer); // Debug print
        break; // Exit the loop after transmitting data
      } else {
        Serial.println("GPS fix lost."); // Debug print
        break; // Exit the loop if no fix
      }
    }
  }

  Serial.println("GPS Data Check Complete."); // Debug print
}


// Log BNO055 Sensor Data
void logBNO055Data() {
  uint8_t system_status, self_test_results, system_error;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);
  /*
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
  */
}

void logDataToSD() {
  // Get the current timestamp
  String timeStamp = getTimeStamp(); // Replace with your method to get the timestamp

  // Get data from BNO055
  imu::Vector<3> bno055Accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  imu::Vector<3> bno055Gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  imu::Vector<3> bno055Mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  int8_t bno055Temp = bno.getTemp();

  // Log BNO055 data
  if (bno055File.open(BNO055_FILE_NAME, O_WRITE | O_APPEND)) {
    bno055File.print(timeStamp + ", ");
    bno055File.print(String(bno055Accel.x()) + ", " + String(bno055Accel.y()) + ", " + String(bno055Accel.z()) + ", ");
    bno055File.print(String(bno055Gyro.x()) + ", " + String(bno055Gyro.y()) + ", " + String(bno055Gyro.z()) + ", ");
    bno055File.print(String(bno055Mag.x()) + ", " + String(bno055Mag.y()) + ", " + String(bno055Mag.z()) + ", ");
    bno055File.println(String(bno055Temp));
    bno055File.sync();
    bno055File.close();
  }

  // Get data from LSM6DSO32
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temp;
  lsm6dso32.getEvent(&accel, &gyro, &temp);
  float lsm6dso32AccelX = accel.acceleration.x;
  float lsm6dso32AccelY = accel.acceleration.y;
  float lsm6dso32AccelZ = accel.acceleration.z;
  float lsm6dso32GyroX = gyro.gyro.x;
  float lsm6dso32GyroY = gyro.gyro.y;
  float lsm6dso32GyroZ = gyro.gyro.z;

  // Log LSM6DSO32 data
  if (lsm6dso32File.open(LSM6DSO32_FILE_NAME, O_WRITE | O_APPEND)) {
    lsm6dso32File.print(timeStamp + ", ");
    lsm6dso32File.println(String(lsm6dso32AccelX) + ", " + String(lsm6dso32AccelY) + ", " + String(lsm6dso32AccelZ) + ", ");
    lsm6dso32File.println(String(lsm6dso32GyroX) + ", " + String(lsm6dso32GyroY) + ", " + String(lsm6dso32GyroZ));
    lsm6dso32File.sync();
    lsm6dso32File.close();
  }

  // Get data from MPL3115A2
  float mplPressure = baro.getPressure();
  float mplAltitude = baro.getAltitude();
  float mplTemperature = baro.getTemperature();

  // Get data from DFRobot_LIS
  float dfLisAccelX = acce.readAccX();
  float dfLisAccelY = acce.readAccY();
  float dfLisAccelZ = acce.readAccZ();

  // Log Other data
  if (otherFile.open(OTHER_FILE_NAME, O_WRITE | O_APPEND)) {
    otherFile.print(timeStamp + ", ");
    otherFile.print(String(mplPressure) + ", " + String(mplAltitude) + ", " + String(mplTemperature) + ", ");
    otherFile.println(String(dfLisAccelX) + ", " + String(dfLisAccelY) + ", " + String(dfLisAccelZ));
    otherFile.sync();
    otherFile.close();
  }
}



void logDF_RobotData(){
  float ax, ay, az;
  ax = acce.readAccX();//Get the acceleration in the x direction
  ay = acce.readAccY();//Get the acceleration in the y direction
  az = acce.readAccZ();//Get the acceleration in the z direction

  Serial.print("x: ");
  Serial.print(ax);
  Serial.print(" g\t  y: ");
  Serial.print(ay);
  Serial.print(" g\t  z: ");
  Serial.print(az);
  Serial.println(" g");
}


// Log MPL3115A2 Sensor Data to Serial Monitor
void logMPL3115A2Data() {
  float pressure = baro.getPressure();   // Pressure in Pascals
  float altitude = baro.getAltitude();   // Altitude in meters
  float temperature = baro.getTemperature(); // Temperature in degrees Celsius

  //Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" Pa");
  //Serial.print("Altitude: "); Serial.print(altitude); Serial.println(" m");
  //Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" C");
}

void rawGPS() {
  static String nmeaSentence = ""; // Buffer to hold NMEA sentence
  // Check if new data is available from the GPS module
  while (GPS.available() > 0) {
    char c = GPS.read();  // Read a byte of the serial data
    
    // Check if the character is the end of a sentence
    if (c == '\n') {
      // Append the character to complete the sentence
      nmeaSentence += c;
      
      // Print the full NMEA sentence to the serial
      Serial.print(nmeaSentence);
      
      // Clear the buffer for the next sentence
      nmeaSentence = "";
    } else {
      // Append the character to the buffer
      nmeaSentence += c;
    }
  }
}
