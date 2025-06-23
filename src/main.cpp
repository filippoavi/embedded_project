#include <Arduino.h>
#include "sd_card.h"
#include "sensor_manager.h"
#include "rtc.h"
//#include "usb.h"

#define DISABLE_FS_H_WARNING  // Disable warning for type File not defined.

// SD card ---------------------------------------------------------------------
bool sdDiagnosticCheck = false; // Perform SD card diagnostics
bool initializeData = true; // If true: delete and reinitialize data.csv file
//------------------------------------------------------------------------------

// Sensors ---------------------------------------------------------------------
unsigned long sensorUpdateInterval = 5000; // 5 seconds 
unsigned long accelerationPollInterval = 100; // 10 Hz
static float maxAccelMag = 0;
static float accelMagnitude;
static float maxGyroMag = 0;
static float gyroMagnitude;
static float maxAccX = 0;
static float maxAccY = 0;
static float maxAccZ = 0;
static float curAccX;
static float curAccY;
static float curAccZ;
static float maxGyrX = 0;
static float maxGyrY = 0;
static float maxGyrZ = 0;
static float curGyrX;
static float curGyrY;
static float curGyrZ;
bool useEvents = true; // Toggle events usage
bool bsecDebug = false;
//------------------------------------------------------------------------------

// Event memory ----------------------------------------------------------------
int accTreshold = 6000;
int gyrTreshold = 200;
const int eventMemorySize = 10;
static int toSave = 0;
static int16_t eventMemory[6][eventMemorySize];
static String eventTime[eventMemorySize];
static int memoryIndex = 0;
static int eventCounter = 0;
bool eventDetected = false;
bool savingEvent = false;
static String curTime;
static String saveBuffer = "";
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
void initEventMemory() {
  for(int i = 0; i < 6; i++) {
    for(int j = 0; j < eventMemorySize; j++) {
      eventMemory[i][j] = 0;
    }
  } 
}
//------------------------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  // Wait for USB Serial
  while (!Serial) {
    yield();
  }

  // Disable all SPI devices before initialization
  pinMode(sensorCS, OUTPUT);
  digitalWrite(sensorCS, HIGH);
  pinMode(SD_CS_PIN, OUTPUT);
  digitalWrite(SD_CS_PIN, HIGH);
  delay(1000);

  // Initialize sensors, SD card and RTC
  sdSetup();
  sensorSetup();
  rtcSetup();

  // Initialize event memory to 0
  initEventMemory();

  // SD card check
  if (sdDiagnosticCheck) {
    sdCheck();
    sdTestWrite();
  }

  // Initialize the CSV file in the SD card
  if(initializeData){
    Serial.print("Initializing data.csv and event.csv...");
    // Remove previous data.csv file
    if (myFile.open("data.csv", O_RDWR)) {
      if (!myFile.remove()) sd.errorHalt("Error data.csv file.remove");
    }
    // Remove previous event.csv file
    if (myFile.open("event.csv", O_RDWR)) {
      if (!myFile.remove()) sd.errorHalt("Error event.csv file.remove");
    }
    // Create new data.csv file
    if (!myFile.open("data.csv", O_RDWR | O_CREAT | O_AT_END)) {
      sd.errorHalt("\nopening data.csv for write failed");
    }
    myFile.println("time,Acceleration_X,Acceleration_Y,Acceleration_Z,Gyroscope_X,Gyroscope_Y,Gyroscope_Z,Magnetometer_X,Magnetometer_Y,Magnetometer_Z,Temperature,Pressure,VOC,CO2,Humidity");
    myFile.close();
    // Create new event.csv file
    if (!myFile.open("event.csv", O_RDWR | O_CREAT | O_AT_END)) {
      sd.errorHalt("\nopening event.csv for write failed");
    }
    myFile.println("time,Acceleration_X,Acceleration_Y,Acceleration_Z,Gyroscope_X,Gyroscope_Y,Gyroscope_Z");
    myFile.close();
    Serial.println("done.");
  }
}
//------------------------------------------------------------------------------
void loop() {
  static auto readTime = millis();
  static auto accelTime = millis();

  // Update function should be continuously polled
  sensorUpdate();

  // Check sensor values every sensorUpdateInterval milliseconds
  if (millis() - readTime >= sensorUpdateInterval) {
    readTime = millis();

    // Write sensor data to SD card
    String line = rtcReadTime() + "," +
                  String(maxAccX) + "," + String(maxAccY) + "," + String(maxAccZ) + "," +
                  String(maxGyrX) + "," + String(maxGyrY) + "," + String(maxGyrZ) + "," +
                  sensorReadMagX() + "," + sensorReadMagY() + "," + sensorReadMagZ() + "," +
                  sensorReadTemperature() + "," + sensorReadPressure() + "," +
                  sensorReadVOC() + "," + sensorReadCO2() + "," + sensorReadHumidity();
    
    Serial.println(line);
    sdWrite(line, "data.csv");
    // So typically a line of the CSV takes more or less 80-100 bytes of memory
    // 1 reading every 5 seconds for 60 days is 100-150 megabytes
    

    // DEBUG BSEC
    if (bsecDebug) {
      Serial.print("BSEC: VOC=");
      Serial.print(bsec.b_voc_eq());
      Serial.print(" CO2=");
      Serial.print(bsec.co2_eq());
      Serial.print(" Humidity=");
      Serial.print(bsec.comp_h());
      Serial.print(" Temperature=");
      Serial.print(bsec.comp_t());
      Serial.print(" Accuracy=");
      Serial.println(bsec.accuracy());
    }

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

  // Poll acceleration and rotation speed every accelerationPollInterval milliseconds
  if(millis() - accelTime >= accelerationPollInterval) {
    accelTime = millis();
    curAccX = sensorGetAccX();
    curAccY = sensorGetAccY();
    curAccZ = sensorGetAccZ();
    curGyrX = sensorGetGyroX();
    curGyrY = sensorGetGyroY();
    curGyrZ = sensorGetGyroZ();
    curTime = rtcReadTime();
    accelMagnitude = sqrt(curAccX * curAccX + curAccY * curAccY + curAccZ * curAccZ);
    gyroMagnitude = sqrt(curGyrX * curGyrX + curGyrY * curGyrY + curGyrZ * curGyrZ);
    // Memorize the maximum value of acceleration and rotation speed of the last 5 seconds
    if(accelMagnitude > maxAccelMag) {
      maxAccelMag = accelMagnitude;
      maxAccX = curAccX;
      maxAccY = curAccY;
      maxAccZ = curAccZ;
    }
    if(gyroMagnitude > maxGyroMag) {
      maxGyroMag = gyroMagnitude;
      maxGyrX = curGyrX;
      maxGyrY = curGyrY;
      maxGyrZ = curGyrZ;
    }

    // Keep track of the last eventMemorySize acceleration and rotation speed values
    eventTime[memoryIndex] = curTime;
    eventMemory[0][memoryIndex] = curAccX;
    eventMemory[1][memoryIndex] = curAccY;
    eventMemory[2][memoryIndex] = curAccZ;
    eventMemory[3][memoryIndex] = curGyrX;
    eventMemory[4][memoryIndex] = curGyrY;
    eventMemory[5][memoryIndex] = curGyrZ;

    // DEBUG: Print the current values to the Serial Monitor
    Serial.println("------------------------------------------------");
    Serial.print("   Index: ");
    Serial.print(memoryIndex);
    Serial.print("   Time: ");
    Serial.print(eventTime[memoryIndex]);
    Serial.print(" Acceleration: [");
    Serial.print(eventMemory[0][memoryIndex]);
    Serial.print(", ");
    Serial.print(eventMemory[1][memoryIndex]);
    Serial.print(", ");
    Serial.print(eventMemory[2][memoryIndex]);
    Serial.print("] Gyroscope: [");
    Serial.print(eventMemory[3][memoryIndex]);
    Serial.print(", ");
    Serial.print(eventMemory[4][memoryIndex]);
    Serial.print(", ");
    Serial.print(eventMemory[5][memoryIndex]);
    Serial.println("]");

    // EVENT HANDLING LOGIC --------------------------------------------------------
    if(memoryIndex < eventMemorySize - 1) {
      memoryIndex++;
    }
    else {
      memoryIndex = 0;
    }
    // If acceleration or rotation speed over a certain treshold, save past to the SD card and keep track of the next values
    if(useEvents && (accelMagnitude > accTreshold || gyroMagnitude > gyrTreshold) && !savingEvent) {
      // Save previous eventMemorySize values before the event
      String lines = "";
      for(int i = 0; i < eventMemorySize; i++) {
        lines += eventTime[(i + memoryIndex)%eventMemorySize] + "[past" + String(i) + "]" +
                      String(eventMemory[0][(i + memoryIndex)%eventMemorySize]) + "," +
                      String(eventMemory[1][(i + memoryIndex)%eventMemorySize]) + "," + 
                      String(eventMemory[2][(i + memoryIndex)%eventMemorySize]) + "," +
                      String(eventMemory[3][(i + memoryIndex)%eventMemorySize]) + "," + 
                      String(eventMemory[4][(i + memoryIndex)%eventMemorySize]) + "," +
                      String(eventMemory[5][(i + memoryIndex)%eventMemorySize]) + "\n";
      }
      saveBuffer += lines;
      savingEvent = true;
      eventCounter = 0;
      toSave = eventMemorySize;
    }
    // If event is detected while already saving values, keep track of more values
    else if(useEvents && (accelMagnitude > accTreshold || gyroMagnitude > gyrTreshold) && savingEvent) {
      toSave = eventMemorySize; // check!
    }

    // Save every eventMemorySize values to keep track of the future after the event
    if (useEvents && savingEvent && eventCounter % eventMemorySize == 0 && eventCounter != 0) {
      // Save previous eventMemorySize values before the event
      String lines = "";
      for(int i = 0; i < eventMemorySize; i++) {
        lines += eventTime[(i + memoryIndex + 1) % eventMemorySize] + "," +
                      String(eventMemory[0][(i + memoryIndex + 1) % eventMemorySize]) + "," +
                      String(eventMemory[1][(i + memoryIndex + 1) % eventMemorySize]) + "," + 
                      String(eventMemory[2][(i + memoryIndex + 1) % eventMemorySize]) + "," +
                      String(eventMemory[3][(i + memoryIndex + 1) % eventMemorySize]) + "," + 
                      String(eventMemory[4][(i + memoryIndex + 1) % eventMemorySize]) + "," +
                      String(eventMemory[5][(i + memoryIndex + 1) % eventMemorySize]) + "\n";
      }
      saveBuffer += lines;
    }

    // Keep track of how many values have been polled after the event
    if(useEvents && savingEvent && toSave > 0) {
      toSave -= 1;
      eventCounter += 1;
    }
    else if (useEvents && savingEvent && toSave <= 0) {
      savingEvent = false;
      String lines = "";
      // We have to save what we have recorded so far, less than eventMemorySize values
      for(int i = - (eventCounter % eventMemorySize) + 1; i < 1; i++) {
        lines += eventTime[(i + memoryIndex) % eventMemorySize] + "," +
                      String(eventMemory[0][(i + memoryIndex) % eventMemorySize]) + "," +
                      String(eventMemory[1][(i + memoryIndex) % eventMemorySize]) + "," + 
                      String(eventMemory[2][(i + memoryIndex) % eventMemorySize]) + "," +
                      String(eventMemory[3][(i + memoryIndex) % eventMemorySize]) + "," + 
                      String(eventMemory[4][(i + memoryIndex) % eventMemorySize]) + "," +
                      String(eventMemory[5][(i + memoryIndex) % eventMemorySize]) + "\n";
      }
      saveBuffer += lines;

      sdWrite(saveBuffer, "event.csv");
      Serial.println("Saving data!");
      Serial.println(saveBuffer);

      saveBuffer = ""; // Reset the save buffer
    }
    //------------------------------------------------------------------------------
  }

  //usbLoop();
}