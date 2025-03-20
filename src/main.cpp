#include "Nicla_System.h"
#include "sd_card.h"
#include "sensors.h"
#define DISABLE_FS_H_WARNING  // Disable warning for type File not defined.

// SD card ---------------------------------------------------------------------
bool sdDiagnosticCheck = false; // Perform SD card diagnostics
bool initializeData = true; // If true: delete and reinitialize data.csv file
//------------------------------------------------------------------------------

// Sensors ---------------------------------------------------------------------
unsigned long sensorUpdateInterval = 5000; // 5 seconds 
static float maxAccelMag = 0;
static float accelMagnitude;
static float maxGyroMag = 0;
static float gyroMagnitude;
static int16_t maxAccX = 0;
static int16_t maxAccY = 0;
static int16_t maxAccZ = 0;
static int16_t curAccX;
static int16_t curAccY;
static int16_t curAccZ;
static int16_t maxGyrX = 0;
static int16_t maxGyrY = 0;
static int16_t maxGyrZ = 0;
static int16_t curGyrX;
static int16_t curGyrY;
static int16_t curGyrZ;
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void setup() {
  nicla::begin();
  nicla::leds.begin();

  Serial.begin(9600);
  // Wait for USB Serial
  while (!Serial) {
    yield();
  }

  // Initialize sensors and SD card
  sensorSetup();
  sdSetup();

  // Signal startup completed through LED blink
  nicla::leds.setColor(green);
  delay(1000);
  nicla::leds.setColor(off);

  // SD card check
  if (sdDiagnosticCheck) {
    sdCheck();
    sdTestWrite();
  }

  // Initialize the CSV file in the SD card
  if(initializeData){
    Serial.print("Initializing data.csv...");
    if (!myFile.open("data.csv", O_RDWR)) {
      sd.errorHalt("\nopening data.csv for remove failed");
    }
    if (!myFile.remove()) sd.errorHalt("Error data.csv file.remove");
    if (!myFile.open("data.csv", O_RDWR | O_CREAT | O_AT_END)) {
      sd.errorHalt("\nopening data.csv for write failed");
    }
    myFile.println("acc_X,acc_Y,acc_Z,gyro_X,gyro_Y,gyro_Z,mag_X,mag_Y,mag_Z,temperature,pressure,VOC,CO2,Humidity");
    myFile.close();
    Serial.println("done.");
  }
}
//------------------------------------------------------------------------------
void loop() {
  static auto printTime = millis();
  static auto accelTime = millis();

  // Update function should be continuously polled
  BHY2.update();

  // Check sensor values every sensorUpdateInterval milliseconds
  if (millis() - printTime >= sensorUpdateInterval) {
    printTime = millis();
    //sensorReadSerial();

    // Write sensor data to SD card
    String line = String(maxAccX) + "," + String(maxAccY) + "," + String(maxAccZ) + "," +
                  String(maxGyrX) + "," + String(maxGyrY) + "," + String(maxGyrZ) + "," +
                  sensorReadMagX() + "," + sensorReadMagY() + "," + sensorReadMagZ() + "," +
                  sensorReadTemperature() + "," + sensorReadPressure() + "," +
                  sensorReadVOC() + "," + sensorReadCO2() + "," + sensorReadHumidity();
    sdWrite(line);
    // So typically a line of the CSV takes more or less 80 bytes of memory
    // 1 reading every 5 seconds for 60 days is 82.944.000 bytes, so 0.1 gigabytes
    Serial.println(line);

    // Reset the maximum acceleration and rotation speed value
    maxAccelMag = 0;
    maxAccX = 0;
    maxAccY = 0;
    maxAccZ = 0;
    maxGyroMag = 0;
    maxGyrX = 0;
    maxGyrY = 0;
    maxGyrZ = 0;
  }

  // Check the magnitude of the acceleration and rotation speed at 100 Hz and memorize the maximum value
  if(millis() - accelTime >= 10) {
    accelTime = millis();
    curAccX = accel.x();
    curAccY = accel.y();
    curAccZ = accel.z();
    curGyrX = gyro.x();
    curGyrY = gyro.y();
    curGyrZ = gyro.z();
    accelMagnitude = sqrt(curAccX * curAccX + curAccY * curAccY + curAccZ * curAccZ);
    gyroMagnitude = sqrt(curGyrX * curGyrX + curGyrY * curGyrY + curGyrZ * curGyrZ);
    if(accelMagnitude > maxAccelMag) {
      maxAccelMag = accelMagnitude;
      maxAccX = curAccX;
      maxAccY = curAccY;
      maxAccZ = curAccZ;
/*       Serial.println("New max acceleration detected: ");
      Serial.println(maxAccelMag); */
    }
    if(gyroMagnitude > maxGyroMag) {
      maxGyroMag = gyroMagnitude;
      maxGyrX = curGyrX;
      maxGyrY = curGyrY;
      maxGyrZ = curGyrZ;
/*       Serial.println("New max rotation speed detected: ");
      Serial.println(maxGyroMag); */
    }
  }
}