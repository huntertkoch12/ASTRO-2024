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
  
  if (dataFile.open("datalog.txt", O_WRITE | O_CREAT | O_APPEND)) {
  dataFile.println("Time, Pitch, Roll, Yaw, AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ, Pressure, Altitude, Temperature");
  dataFile.close();
  Serial.println("Data log started.");
} else {
  Serial.println("Error opening datalog.txt!");
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
}

// Initialize the GPS
void initGPS() {
  GPS.begin(9600);
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
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
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

// Receive Message and Transmit Reply
void transmitGPSData() {
  static unsigned long lastTime = 0;
  static const unsigned long timeout = 3000;  // 3 seconds timeout

  // Check if new NMEA data is available
  while (GPS.available()) {
    char c = GPS.read();
    if (GPS.newNMEAreceived()) {
      if (!GPS.parse(GPS.lastNMEA())) return;
    }
  }

  // Check if we have a GPS fix
  if (GPS.fix) {
    Serial.print("Location: ");
    Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
    Serial.print(", ");
    Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);

    // Prepare the GPS data into a transmission buffer
    char transmitBuffer[50];
    snprintf(transmitBuffer, sizeof(transmitBuffer), "Lat:%f%s, Lon:%f%s", 
             GPS.latitude, GPS.lat == 'N' ? "N" : "S", 
             GPS.longitude, GPS.lon == 'E' ? "E" : "W");
    
    // Transmit the GPS data
    rf95.send((uint8_t *)transmitBuffer, strlen(transmitBuffer) + 1); // +1 to include null-terminator
    rf95.waitPacketSent(); // Wait until the packet is sent
    
    // Turn on the built-in LED after transmitting
    digitalWrite(LED_BUILTIN, HIGH);

    Serial.println("GPS data transmitted!");

    lastTime = millis();
  } else if (millis() - lastTime > timeout) {
    // Turn off the built-in LED if we don't get a fix
    digitalWrite(LED_BUILTIN, LOW);
  }
}



// Log BNO055 Sensor Data
void logBNO055Data() {
  uint8_t system_status, self_test_results, system_error;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

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

void logDataToSD() {
  // Open the file
  if (!dataFile.open("datalog.txt", O_WRITE | O_CREAT | O_APPEND)) {
    Serial.println("Error opening datalog.txt");
    return;
  }

  // If the file is open, write to it
  if (dataFile) {
    // Get the timestamp from the RTC
    String timeStamp = getTimeStamp();

    // Get BNO055 data
    imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
    imu::Vector<3> accel = bno.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
    imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
    imu::Vector<3> mag = bno.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);

    // Get MPL3115A2 data
    float pressure = baro.getPressure();
    float altitude = baro.getAltitude();
    float temperature = baro.getTemperature();

    // Write data to SD card
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
