#include "Nicla_System.h"
//#include "ArduinoLowPower.h"  // Power management (not working for this board)
#include "sd_card.h"
#include "sensors.h"
#define DISABLE_FS_H_WARNING  // Disable warning for type File not defined.

// SD card ---------------------------------------------------------------------
bool sdDiagnosticCheck = false; // Perform SD card diagnostics
bool initializeData = true; // If true: delete and reinitialize data.csv file
//------------------------------------------------------------------------------

// Sensors ---------------------------------------------------------------------
unsigned long sensorUpdateInterval = 5000; // 5 seconds 
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
    // Test writing to SD card
    if (!myFile.open("test.txt", O_RDWR | O_CREAT | O_AT_END)) {
      sd.errorHalt("\nopening test.txt for write failed");
    }
    // if the file opened okay, write to it:
    Serial.print("\nWriting to test.txt...");
    myFile.println("If you are reading this, writing and reading to the SD card works!");
    // close the file:
    myFile.close();
    Serial.println("done.");
    // re-open the file for reading:
    if (!myFile.open("test.txt", O_RDWR)) {
      sd.errorHalt("opening test.txt for read failed");
    }
    Serial.println("test.txt:");

    // read from the file until there's nothing else in it:
    int data;
    while ((data = myFile.read()) >= 0) {
      Serial.write(data);
    }
    // remove the file:
    if (!myFile.remove()) sd.errorHalt("Error file.remove");
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

  // Update function should be continuously polled
  BHY2.update();

  // Check sensor values every sensorUpdateInterval milliseconds
  if (millis() - printTime >= sensorUpdateInterval) {
    printTime = millis();
    sensorReadSerial();

    // Write sensor data to SD card
    String line = sensorReadAccX() + "," + sensorReadAccY() + "," + sensorReadAccZ() + "," +
                  sensorReadGyroX() + "," + sensorReadGyroY() + "," + sensorReadGyroZ() + "," +
                  sensorReadMagX() + "," + sensorReadMagY() + "," + sensorReadMagZ() + "," +
                  sensorReadTemperature() + "," + sensorReadPressure() + "," +
                  sensorReadVOC() + "," + sensorReadCO2() + "," + sensorReadHumidity();
    sdWrite(line);
  }
}