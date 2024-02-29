//*******************************************//
//             Include Libraries             //
//*******************************************//

#include "config.h"

//*******************************************//
//       Create Sensor & Radio Instances     //
//*******************************************//

RH_RF95 rf95(RFM95_CS, RFM95_INT);
Adafruit_MPL3115A2 baro = Adafruit_MPL3115A2();
Adafruit_GPS GPS(&Serial1); // Assuming you are using UART1 for GPS
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);
RTC_DS3231 rtc;
Adafruit_LSM6DSO32 lsm6dso32;
ScioSense_ENS160 ens160(0x53);
Adafruit_BME680 bme;

//*******************************************//
//            Initialization Block           //
//*******************************************//

// BNO055

void bno_write(uint8_t i2c_addr, uint8_t reg, uint8_t data) // write one BNO register
{
  Wire.beginTransmission(i2c_addr);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission(true); // send stop
}

void bno_read_multiple(uint8_t i2c_addr, uint8_t reg, uint8_t *buf, uint8_t length) // read multiple BNO registers into buffer
{
  for (uint32_t n = 0; n < length; n++)
  {
    if ((n & 31) == 0) // transfer up to 32 bytes at a time
    {
      Wire.beginTransmission(i2c_addr);
      Wire.write(reg + n);
      Wire.endTransmission(false); // send restart
      Wire.requestFrom(i2c_addr, min(length - n, 32));
    }
    *buf++ = Wire.read();
  }
}

// BNO055

// Initialize LoRa Radio
void initRadio()
{

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

  if (!rf95.init())
  {
    Serial.println("LoRa initialization failed!");
    Serial.println("Uncomment '#define SERIAL_DEBUG' in RH_RF95.cpp for detailed debug info!");
    while (1)
    {
      strip.setPixelColor(0, strip.Color(255, 0, 0)); // Flash red to indicate error
      strip.show();
      delay(250);
      strip.clear();
      delay(250);
    }
  }

  Serial.println("LoRa initialization successful!");

  if (!rf95.setFrequency(RF95_FREQ))
  {
    Serial.println("Failed to set LoRa frequency!");
    while (1)
    {
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
void initBNO055()
{
  Serial.begin(115200);

  digitalWrite(RST, 0);
  pinMode(RST, OUTPUT); // assert BNO RST
  delay(1);
  pinMode(RST, INPUT_PULLUP); // deassert BNO RST
  delay(800);                 // allow time for BNO to boot

  bno_write(BNO_ADDR, PAGE_ID, 1);         // register page 1
  bno_write(BNO_ADDR, ACC_CONFIG, 0x0F);   // accel +/-16g range (default value 0x0D)
  bno_write(BNO_ADDR, PAGE_ID, 0);         // register page 0
  bno_write(BNO_ADDR, OPR_MODE, MODE_AMG); // operating mode
  delay(10);                               // allow time for BNO to switch modes

  // Set the BNO055 accelerometer to ±16g range
  bno_write(BNO_ADDR, PAGE_ID, 1);       // Switch to register page 1
  bno_write(BNO_ADDR, ACC_CONFIG, 0x0F); // Set accelerometer range to ±16g
  Serial.println(BNO_ADDR);
  Serial.println(ACC_CONFIG);
  bno_write(BNO_ADDR, PAGE_ID, 0); // Switch back to register page 0
}

void initLSM6DSO32()
{
  Serial.println("LSM6DSO32 Sensor Test");

  if (!lsm6dso32.begin_I2C())
  {
    Serial.println("Failed to find LSM6DSO32 chip");
    while (1)
    {
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

void handleError()
{
  while (1)
  {
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

const char *BNO055_FILE_NAME = "BNO055Data.csv";
const char *LSM6DSO32_FILE_NAME = "LSM6DSO32Data.csv";
const char *OTHER_FILE_NAME = "OtherData.csv";

void setupSDCard()
{
  Serial.print("Initializing SD card...");

  if (!sd.begin(chipSelect, SPI_FULL_SPEED))
  {
    Serial.println("Initialization failed!");
    while (1)
    {
      strip.setPixelColor(0, strip.Color(255, 0, 0)); // Flash red to indicate error
      strip.show();
      delay(250);
      strip.clear();
      delay(250);
    }
  }
  Serial.println("Initialization done.");

  // Initialize BNO055 data file
  if (!bno055File.open(BNO055_FILE_NAME, O_WRITE | O_CREAT | O_APPEND))
  {
    Serial.println("Error opening BNO055 data file!");
    handleError();
  }
  // Write headers for BNO055 file
  bno055File.println("Timestamp, AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ, MagX, MagY, MagZ");
  bno055File.close();

  // Initialize LSM6DSO32 data file
  if (!lsm6dso32File.open(LSM6DSO32_FILE_NAME, O_WRITE | O_CREAT | O_APPEND))
  {
    Serial.println("Error opening LSM6DSO32 data file!");
    handleError();
  }
  // Write headers for LSM6DSO32 file
  lsm6dso32File.println("Timestamp, AccelX, AccelY, AccelZ, GyroX, GyroY, GyroZ");
  lsm6dso32File.close();

  // Initialize Other data file
  if (!otherFile.open(OTHER_FILE_NAME, O_WRITE | O_CREAT | O_APPEND))
  {
    Serial.println("Error opening Other data file!");
    handleError();
  }
  // Write headers or other initialization data for Other file
  otherFile.println("Timestamp, MPL_Pressure, MPL_Altitude, MPL_Temperature, BME_Temp, BME_Humidity, BME_Pressure, BME_Gas, BME_Altitude, ENS_TVOC, ENS_eCO2, ENS_AQI, ENS_HP0, ENS_HP1, ENS_HP2, ENS_HP3");
  otherFile.close();

  Serial.println("All data logs started.");
}

// Initalize Altimeter
void initMPL3115A2()
{
  Serial.println("Initializing MPL3115A2 Sensor");

  if (!baro.begin())
  {
    Serial.println("Could not find a valid MPL3115A2 sensor, check wiring!");
    while (1)
    {
      strip.setPixelColor(0, strip.Color(255, 0, 0)); // Flash red to indicate error
      strip.show();
      delay(250);
      strip.clear();
      delay(250);
    }
  }

  // Set a different oversampling rate
  // baro.setOversamplingRate(MPL3115A2_CTRL_REG1_OS128);  // Example: Set to OS4

  Serial.println("MPL3115A2 Sensor initialized with custom sampling rate.");
}

// Initialize the GPS
void initGPS()
{
  GPS.begin(9600);
  // Flush any old data from the GPS serial buffer
  while (GPS.available() > 0)
  {
    GPS.read();
  }
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);
  GPS.sendCommand(PGCMD_ANTENNA);
}

// Initalize RTC
void setupRTC()
{

  if (!rtc.begin())
  {
    Serial.println("Couldn't find RTC!");
    while (1)
    {
      strip.setPixelColor(0, strip.Color(255, 0, 0)); // Flash red to indicate error
      strip.show();
      delay(250);
      strip.clear();
      delay(250);
    }
  }

  if (rtc.lostPower())
  {
    Serial.println("RTC lost power, please set the time!");
    // Manually set the date and time
    // rtc.adjust(DateTime(2023, 1, 21, 3, 0, 0));
    // Or use the compile time: (this will not be accurate if the RTC has lost power)
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
}

void initENS160()
{
  // Attempt to initialize the ENS160 sensor
  ens160.begin();

  // Check if the sensor is available and successfully initialized
  if (ens160.available())
  {
    // Sensor initialized successfully, you can proceed with further configurations or readings
    Serial.println("ENS160 initialization successful!");

    // Optionally, print sensor version information
    Serial.print("\tRev: ");
    Serial.print(ens160.getMajorRev());
    Serial.print(".");
    Serial.print(ens160.getMinorRev());
    Serial.print(".");
    Serial.println(ens160.getBuild());

    // Set the sensor to standard operating mode
    Serial.print("\tStandard mode ");
    Serial.println(ens160.setMode(ENS160_OPMODE_STD) ? "done." : "failed!");
  }
  else
  {
    // Sensor initialization failed, handle the error accordingly
    Serial.println("ENS160 initialization failed!");

    // You can add additional error handling here, such as blinking an LED to indicate failure or retrying initialization
  }
}

void initBME688()
{
  if (!bme.begin())
  {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1)
      ;
  }

  Serial.println("Found the BME680!");

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms
}

void initMicrophone()
{
  // Initialize ADC
  adc_init();

  // Select ADC input 0 (GPIO 26) as the analog source
  adc_gpio_init(A0);

  // Set ADC reference voltage (3.3V for RP2040)
  adc_set_temp_sensor_enabled(false);
}

// Set GPS rate
void setGPSUpdateRate(int milliseconds)
{
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
String getTimeStamp()
{
  DateTime now = rtc.now();
  char buf[] = "YYYY-MM-DD hh:mm:ss";
  snprintf(buf, sizeof(buf), "%04d-%02d-%02d %02d:%02d:%02d", now.year(), now.month(), now.day(), now.hour(), now.minute(), now.second());
  return String(buf);
}

void transmitGPSData()
{
  static unsigned int packetCounter = 0; // Packet counter

  Serial.println("Checking GPS Data..."); // Debug print

  // Check if new NMEA data is available
  while (GPS.available())
  {
    char c = GPS.read();
    Serial.print(c); // Print raw GPS data
    if (GPS.newNMEAreceived())
    {
      Serial.println("New NMEA sentence received."); // Debug print
      if (!GPS.parse(GPS.lastNMEA()))
      {
        Serial.println("Failed to parse NMEA sentence."); // Debug print
        break;                                            // Exit the loop if the sentence can't be parsed
      }

      // Check if we have a GPS fix
      if (GPS.fix)
      {
        Serial.println("GPS fix obtained."); // Debug print
        packetCounter++;                     // Increment the packet counter

        // Prepare the GPS data into a transmission buffer
        char transmitBuffer[60]; // Increased buffer size to accommodate packet counter
        snprintf(transmitBuffer, sizeof(transmitBuffer), "Packet %u, Lat:%f%s, Lon:%f%s",
                 packetCounter,
                 GPS.latitude, GPS.lat == 'N' ? "N" : "S",
                 GPS.longitude, GPS.lon == 'E' ? "E" : "W");

        // Transmit the GPS data
        rf95.send((uint8_t *)transmitBuffer, strlen(transmitBuffer) + 1); // +1 to include null-terminator
        Serial.println(transmitBuffer);                                   // Debug print
        break;                                                            // Exit the loop after transmitting data
      }
      else
      {
        Serial.println("GPS fix lost."); // Debug print
        break;                           // Exit the loop if no fix
      }
    }
  }

  Serial.println("GPS Data Check Complete."); // Debug print
}

// Log BNO055 Sensor Data
void logBNO055Data()
{
  struct
  {
    int16_t acc_x, acc_y, acc_z;
  } s;

  struct
  {
    int16_t mag_x, mag_y, mag_z;
  } m;

  struct
  {
    int16_t gyr_x, gyr_y, gyr_z;
  } g;

  bno_read_multiple(BNO_ADDR, ACC_DATA_X_LSB, (uint8_t *)&s, sizeof s); // read accelerometer data

  bno_read_multiple(BNO_ADDR, MAG_DATA_X_LSB, (uint8_t *)&m, sizeof m); // read magnetometer data

  bno_read_multiple(BNO_ADDR, GYR_DATA_X_LSB, (uint8_t *)&g, sizeof g); // read gyroscope data

  // Print accelerometer data
  Serial.print("Accelerometer (100 * m/s^2): ");
  Serial.print(s.acc_x);
  Serial.print(" ");
  Serial.print(s.acc_y);
  Serial.print(" ");
  Serial.print(s.acc_z);
  Serial.println("");

  // Print magnetometer data
  Serial.print("Magnetometer: ");
  Serial.print(m.mag_x);
  Serial.print(" ");
  Serial.print(m.mag_y);
  Serial.print(" ");
  Serial.print(m.mag_z);
  Serial.println("");

  // Print gyroscope data
  Serial.print("Gyro: ");
  Serial.print(g.gyr_x);
  Serial.print(" ");
  Serial.print(g.gyr_y);
  Serial.print(" ");
  Serial.print(g.gyr_z);
  Serial.println("");

  delay(100);
}

void logDataToSD()
{
  struct
  {
    int16_t acc_x, acc_y, acc_z;
  } s;

  struct
  {
    int16_t mag_x, mag_y, mag_z;
  } m;

  struct
  {
    int16_t gyr_x, gyr_y, gyr_z;
  } g;

  // Read data from BNO055
  bno_read_multiple(BNO_ADDR, ACC_DATA_X_LSB, (uint8_t *)&s, sizeof s);
  bno_read_multiple(BNO_ADDR, MAG_DATA_X_LSB, (uint8_t *)&m, sizeof m);
  bno_read_multiple(BNO_ADDR, GYR_DATA_X_LSB, (uint8_t *)&g, sizeof g);

  // Get the current timestamp
  String timeStamp = getTimeStamp(); // Replace with your method to get the timestamp

  // Log BNO055 data to SD card
  if (bno055File.open(BNO055_FILE_NAME, O_WRITE | O_APPEND))
  {
    bno055File.print(timeStamp + ", ");
    bno055File.print(String(s.acc_x) + ", " + String(s.acc_y) + ", " + String(s.acc_z) + ", ");
    bno055File.print(String(g.gyr_x) + ", " + String(g.gyr_y) + ", " + String(g.gyr_z) + ", ");
    bno055File.print(String(m.mag_x) + ", " + String(m.mag_y) + ", " + String(m.mag_z));
    bno055File.println();
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
  if (lsm6dso32File.open(LSM6DSO32_FILE_NAME, O_WRITE | O_APPEND))
  {
    lsm6dso32File.print(timeStamp + ", ");
    lsm6dso32File.print(String(lsm6dso32AccelX) + ", " + String(lsm6dso32AccelY) + ", " + String(lsm6dso32AccelZ) + ", ");
    lsm6dso32File.println(String(lsm6dso32GyroX) + ", " + String(lsm6dso32GyroY) + ", " + String(lsm6dso32GyroZ));
    lsm6dso32File.sync();
    lsm6dso32File.close();
  }

  // Get data from MPL3115A2
  float mplPressure = baro.getPressure();
  float mplAltitude = baro.getAltitude();
  float mplTemperature = baro.getTemperature();

  // Log combined data
  if (otherFile.open(OTHER_FILE_NAME, O_WRITE | O_APPEND))
  {
    otherFile.print(timeStamp + ", ");
    otherFile.print(String(mplPressure) + ", " + String(mplAltitude) + ", " + String(mplTemperature) + ", ");

    // Append BME688 data
    otherFile.print(String(bme.temperature) + ", ");
    otherFile.print(String(bme.humidity) + ", ");
    otherFile.print(String(bme.pressure / 100.0) + ", ");
    otherFile.print(String(bme.readAltitude(SEALEVELPRESSURE_HPA)) + ", ");
    otherFile.print(String(bme.gas_resistance / 1000.0) + ", ");

    // Append ENS160 data
    otherFile.print(String(ens160.getTVOC()) + ", ");
    otherFile.print(String(ens160.geteCO2()) + ", ");
    otherFile.print(String(ens160.getAQI()) + ", ");
    otherFile.println(String(ens160.getHP0()) + ", " + String(ens160.getHP1()) + ", " + String(ens160.getHP2()) + ", " + String(ens160.getHP3()));

    otherFile.sync();
    otherFile.close();
  }
}

// Log MPL3115A2 Sensor Data to Serial Monitor
void logMPL3115A2Data()
{
  float pressure = baro.getPressure();       // Pressure in Pascals
  float altitude = baro.getAltitude();       // Altitude in meters
  float temperature = baro.getTemperature(); // Temperature in degrees Celsius

  // Serial.print("Pressure: "); Serial.print(pressure); Serial.println(" Pa");
  // Serial.print("Altitude: "); Serial.print(altitude); Serial.println(" m");
  // Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" C");
}

void logENS160()
{
  if (ens160.available())
  {
    ens160.measure(true);
    ens160.measureRaw(true);

    Serial.print("AQI: ");
    Serial.print(ens160.getAQI());
    Serial.print("\t");
    Serial.print("TVOC: ");
    Serial.print(ens160.getTVOC());
    Serial.print("ppb\t");
    Serial.print("eCO2: ");
    Serial.print(ens160.geteCO2());
    Serial.print("ppm\t");
    Serial.print("R HP0: ");
    Serial.print(ens160.getHP0());
    Serial.print("Ohm\t");
    Serial.print("R HP1: ");
    Serial.print(ens160.getHP1());
    Serial.print("Ohm\t");
    Serial.print("R HP2: ");
    Serial.print(ens160.getHP2());
    Serial.print("Ohm\t");
    Serial.print("R HP3: ");
    Serial.print(ens160.getHP3());
    Serial.println("Ohm");
  }
  delay(1000);
}

void logBME688()
{
  if (!bme.performReading())
  {
    Serial.println("Failed to perform reading :(");
    return;
  }
  Serial.print("Temperature = ");
  Serial.print(bme.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bme.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Humidity = ");
  Serial.print(bme.humidity);
  Serial.println(" %");

  Serial.print("Gas = ");
  Serial.print(bme.gas_resistance / 1000.0);
  Serial.println(" KOhms");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();
  delay(2000);
}

void logMicrophone()
{

  // Select ADC input channel
  adc_select_input(0); // Channel 0 corresponds to GPIO 26 on most boards

  // Read the analog value
  uint16_t analogValue = adc_read();

  // Print the raw analog value
  Serial.println(analogValue);

  // Wait a bit before the next read
  delay(100); // Delay in milliseconds
}

void rawGPS()
{
  static String nmeaSentence = ""; // Buffer to hold NMEA sentence
  // Check if new data is available from the GPS module
  while (GPS.available() > 0)
  {
    char c = GPS.read(); // Read a byte of the serial data

    // Check if the character is the end of a sentence
    if (c == '\n')
    {
      // Append the character to complete the sentence
      nmeaSentence += c;

      // Print the full NMEA sentence to the serial
      Serial.print(nmeaSentence);

      // Clear the buffer for the next sentence
      nmeaSentence = "";
    }
    else
    {
      // Append the character to the buffer
      nmeaSentence += c;
    }
  }
}
