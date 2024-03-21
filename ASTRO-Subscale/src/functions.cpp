//*******************************************//
//             Include Libraries             //
//*******************************************//

#include "config.h"

//*******************************************//
//       Create Sensor & Radio Instances     //
//*******************************************//

Adafruit_NeoPixel strip = Adafruit_NeoPixel(NEOPIXEL_NUM, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

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

  // Log BNO055 data to SD card
  if (bno055File.open(BNO055_FILE_NAME, O_WRITE | O_APPEND))
  {
    bno055File.print(String(s.acc_x) + ", " + String(s.acc_y) + ", " + String(s.acc_z) + ", ");
    bno055File.print(String(g.gyr_x) + ", " + String(g.gyr_y) + ", " + String(g.gyr_z) + ", ");
    bno055File.print(String(m.mag_x) + ", " + String(m.mag_y) + ", " + String(m.mag_z));
    bno055File.println();
    bno055File.sync();
    bno055File.close();
  }
}
