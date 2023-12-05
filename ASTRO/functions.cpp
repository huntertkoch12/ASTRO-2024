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



// Initialize SD Card
const int chipSelect = 10;
SdFat sd;
SdFile dataFile;

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
  
  if (dataFile.open("datalog.csv", O_WRITE | O_CREAT | O_APPEND)) {
    dataFile.println("Time, Pitch, Roll, Yaw, AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ, QuatW, QuatX, QuatY, QuatZ, LinAccX, LinAccY, LinAccZ, GravityX, GravityY, GravityZ, Pressure, Altitude, Temperature, DF_X_Acceleration, DF_Y_Acceleration, DF_Z_Acceleration");
    dataFile.close();
    Serial.println("Data log started.");
  } else {
    Serial.println("Error opening datalog.csv!");
    while (1) {
      strip.setPixelColor(0, strip.Color(255, 0, 0)); // Flash red to indicate error
      strip.show();
      delay(250);
      strip.clear();
      delay(250);
    }
  }
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
  baro.setOversamplingRate(MPL3115A2_CTRL_REG1_OS4); // Example: Set to OS4
  
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
  // Check if the SD card is present
  if (!sd.exists("datalog.csv")) {
    Serial.println("SD card not found or datalog.csv doesn't exist");

    // Set NeoPixel to pink color
    strip.setPixelColor(0, strip.Color(255, 105, 180));
    strip.show();

    return;
  }

  // Open the file
  if (!dataFile.open("datalog.csv", O_WRITE | O_CREAT | O_APPEND)) {
    Serial.println("Error opening datalog.csv");
    return;
  }

  // If the file is open, write to it
  if (dataFile) {
    // Get the timestamp from the RTC
    String timeStamp = getTimeStamp();

    // Get all BNO055 data
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
    imu::Quaternion quat = bno.getQuat();
    imu::Vector<3> linAccel = bno.getVector(Adafruit_BNO055::VECTOR_LINEARACCEL);
    imu::Vector<3> grav = bno.getVector(Adafruit_BNO055::VECTOR_GRAVITY);

    // Get MPL3115A2 data
    float pressure = baro.getPressure();
    float altitude = baro.getAltitude();
    float temperature = baro.getTemperature();

    // Get DF_Robot data
    float df_ax = acce.readAccX();
    float df_ay = acce.readAccY();
    float df_az = acce.readAccZ();

    // Write data to SD card
    // Include all the data fields here in the format you want to save them
    dataFile.print(timeStamp);
    dataFile.print(", ");
    dataFile.print(euler.x()); dataFile.print(", "); dataFile.print(euler.y()); dataFile.print(", "); dataFile.print(euler.z());
    dataFile.print(", ");
    dataFile.print(accel.x()); dataFile.print(", "); dataFile.print(accel.y()); dataFile.print(", "); dataFile.print(accel.z());
    dataFile.print(", ");
    dataFile.print(gyro.x()); dataFile.print(", "); dataFile.print(gyro.y()); dataFile.print(", "); dataFile.print(gyro.z());
    dataFile.print(", ");
    dataFile.print(mag.x()); dataFile.print(", "); dataFile.print(mag.y()); dataFile.print(", "); dataFile.print(mag.z());
    dataFile.print(", ");
    dataFile.print(quat.w()); dataFile.print(", "); dataFile.print(quat.x()); dataFile.print(", "); dataFile.print(quat.y()); dataFile.print(", "); dataFile.print(quat.z());
    dataFile.print(", ");
    dataFile.print(linAccel.x()); dataFile.print(", "); dataFile.print(linAccel.y()); dataFile.print(", "); dataFile.print(linAccel.z());
    dataFile.print(", ");
    dataFile.print(grav.x()); dataFile.print(", "); dataFile.print(grav.y()); dataFile.print(", "); dataFile.print(grav.z());
    dataFile.print(", ");
    dataFile.print(pressure); dataFile.print(", "); dataFile.print(altitude); dataFile.print(", "); dataFile.print(temperature);
    dataFile.print(", ");
    dataFile.print(df_ax); dataFile.print(", "); dataFile.print(df_ay); dataFile.print(", "); dataFile.print(df_az);
    dataFile.print(",\n");

    // Close the file
    dataFile.close();
  } else {
    // If the file didn't open, print an error
    Serial.println("Error opening datalog.csv");
  }

  // Flush the data to make sure it's written to the SD card
  if (!dataFile.sync()) {
    Serial.println("Error: Data not saved properly");
  }

  Serial.println("Finished writing!");
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
