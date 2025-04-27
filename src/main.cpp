//#include "Nicla_System.h"
#include "sd_card.h"
#include "sensors.h"
#include "rtc.h"
//#include "usb.h"

#define DISABLE_FS_H_WARNING  // Disable warning for type File not defined.

// SD card ---------------------------------------------------------------------
bool sdDiagnosticCheck = false; // Perform SD card diagnostics
bool initializeData = true; // If true: delete and reinitialize data.csv file
//------------------------------------------------------------------------------

// Sensors ---------------------------------------------------------------------
struct bhy2_dev bhy2_device;

unsigned long sensorUpdateInterval = 5000; // 5 seconds 
unsigned long accelerationPollInterval = 100; // 10 Hz
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

// Event memory ----------------------------------------------------------------
int accTreshold = 6000;
int gyrTreshold = 200;
const int eventMemorySize = 100;
static int toSave = 0;
static int16_t eventMemory[6][eventMemorySize];
static String eventTime[eventMemorySize];
static int memoryIndex = 0;
static int eventCounter = 0;
bool eventDetected = false;
bool savingEvent = false;
static String curTime;
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
  /* nicla::begin();
  nicla::leds.begin(); */

  Serial.begin(9600);
  // Wait for USB Serial
  while (!Serial) {
    yield();
  }

  // Initialize sensors, SD card and RTC
  sensorSetup(bhy2_device);
  sdSetup();
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

  // Signal startup completed through green LED blink
  /* nicla::leds.setColor(green);
  delay(3000);
  nicla::leds.setColor(off); */
}
//------------------------------------------------------------------------------
void loop() {
  static auto printTime = millis();
  static auto accelTime = millis();

  // Update function should be continuously polled
  //BHY2.update();
  sensorUpdate(bhy2_device);

  // Check sensor values every sensorUpdateInterval milliseconds
  if (millis() - printTime >= sensorUpdateInterval) {
    printTime = millis();
    //sensorReadSerial();

    // Write sensor data to SD card
    String line = rtcReadTime() + "," +
                  String(maxAccX) + "," + String(maxAccY) + "," + String(maxAccZ) + "," +
                  String(maxGyrX) + "," + String(maxGyrY) + "," + String(maxGyrZ) + "," +
                  sensorReadMagX() + "," + sensorReadMagY() + "," + sensorReadMagZ() + "," +
                  sensorReadTemperature() + "," + sensorReadPressure() + "," +
                  /*sensorReadVOC() + "," + sensorReadCO2() + "," + */sensorReadHumidity();
    sdWrite(line, "data.csv");
    // So typically a line of the CSV takes more or less 80-100 bytes of memory
    // 1 reading every 5 seconds for 60 days is 100-150 megabytes
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

    // EVENT HANDLING LOGIC --------------------------------------------------------
    if(memoryIndex < eventMemorySize - 1) {
      memoryIndex++;
    }
    else {
      memoryIndex = 0;
    }
    // If acceleration or rotation speed over a certain treshold, save past to the SD card and keep track of the next values
    if((accelMagnitude > accTreshold || gyroMagnitude > gyrTreshold) && !savingEvent) {
      // Save previous eventMemorySize values before the event
      String lines = "";
      for(int i = 0; i < eventMemorySize; i++) {
        lines += eventTime[(i + memoryIndex)%eventMemorySize] + "[past" + String(i) + "]" +
                      String(eventMemory[0][(i + memoryIndex)%eventMemorySize]) + "," + String(eventMemory[1][(i + memoryIndex)%eventMemorySize]) + "," + 
                      String(eventMemory[2][(i + memoryIndex)%eventMemorySize]) + "," + String(eventMemory[3][(i + memoryIndex)%eventMemorySize]) + "," + 
                      String(eventMemory[4][(i + memoryIndex)%eventMemorySize]) + "," + String(eventMemory[5][(i + memoryIndex)%eventMemorySize]);
      }
      sdWrite(lines, "event.csv");
      Serial.println("Saving data!");
      Serial.println(lines);
      savingEvent = true;
      eventCounter = 0;
      toSave = eventMemorySize;
    }
    // If event is detected while already saving values, keep track of more values
    else if((accelMagnitude > accTreshold || gyroMagnitude > gyrTreshold) && savingEvent) {
      toSave += eventCounter;
    }

    // Save every eventMemorySize values to keep track of the future after the event
    if (savingEvent && eventCounter % eventMemorySize == 0 && eventCounter != 0) {
      // Save previous eventMemorySize values before the event
      String lines = "";
      for(int i = 0; i < eventMemorySize; i++) {
        lines += eventTime[(i + memoryIndex)%eventMemorySize] + "[future" + String(i) +
                      " --- eventCounter=" + String (eventCounter) + " --- toSave=" + String(toSave) + "]" +
                      String(eventMemory[0][(i + memoryIndex)%eventMemorySize]) + "," + String(eventMemory[1][(i + memoryIndex)%eventMemorySize]) + "," + 
                      String(eventMemory[2][(i + memoryIndex)%eventMemorySize]) + "," + String(eventMemory[3][(i + memoryIndex)%eventMemorySize]) + "," + 
                      String(eventMemory[4][(i + memoryIndex)%eventMemorySize]) + "," + String(eventMemory[5][(i + memoryIndex)%eventMemorySize]);
      }
      sdWrite(lines, "event.csv");
      Serial.println("Saving data!");
      Serial.println(lines);
    }

    // Keep track of how many values have been polled after the event
    if(savingEvent && eventCounter < toSave) {
      eventCounter += 1;
    }
    else if (savingEvent && eventCounter > toSave-2) {
      savingEvent = false;
      // We have to save what we have recorded so far, less than eventMemorySize values ... TODO
    }
    //------------------------------------------------------------------------------
  }

  //usbLoop();
}