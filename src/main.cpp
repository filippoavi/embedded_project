#include <Arduino.h>
#include "sd_card.h"
#include "sensor_manager.h"
#include "rtc.h"
#include "load_firmware.h"
//#include "usb.h"

#define DISABLE_FS_H_WARNING  // Disable warning for type File not defined.

// SD card ---------------------------------------------------------------------
bool sdDiagnosticCheck = false; // Perform SD card diagnostics
bool initializeData = true; // If true: delete and reinitialize data.csv file
//------------------------------------------------------------------------------

// Measuremets ---------------------------------------------------
bool readSensors = true;
bool readRTC = false;
bool writeToSD = false;
bool writeToSDEvent = false; // Write to event.csv file
String line = "2025/03/21-00:00:00,1860,-318,4554,441,321,-270,273,-138,-222,24.73,996.55,0.45,500,40.22";
String lineEvent = "2025/03/23-16:09:26,118,-14,4093,-1,-2,1\n2025/03/22-18:05:46,127,-9,4103,-1,1,0\n2025/03/22-16:34:51,114,-12,4081,0,-2,0\n2025/03/23-18:48:58,117,-18,4115,2,-2,2\n2025/03/24-23:21:31,127,-13,4094,-1,-1,2\n2025/03/25-17:23:10,118,-28,4112,-1,-1,-1\n2025/03/21-16:27:48,121,-14,4106,-2,-1,1\n2025/03/23-03:10:35,119,-18,4088,-2,1,1\n2025/03/25-11:54:58,114,-29,4093,2,-2,0\n2025/03/21-11:23:48,109,-2,4089,-2,0,2\n2025/03/25-20:24:09,120,-13,4106,-1,0,0\n2025/03/23-05:54:52,120,-4,4106,-1,-2,0\n2025/03/24-11:23:20,111,-24,4089,-2,1,-1\n2025/03/23-02:38:53,109,-19,4088,-2,-1,-2\n2025/03/24-12:14:16,127,-30,4099,2,1,2\n2025/03/21-10:32:20,112,-25,4106,-2,2,2\n2025/03/22-23:25:31,110,-24,4093,1,-2,-2\n2025/03/24-15:47:41,115,-18,4114,2,-1,-2\n2025/03/25-18:32:48,129,-15,4080,-2,1,-1\n2025/03/21-10:43:52,115,-24,4083,1,2,-1\n2025/03/23-22:22:05,116,-2,4101,0,-2,-1\n2025/03/21-07:30:36,107,-25,4108,-1,-1,-2\n2025/03/22-20:40:22,128,-8,4081,-1,-2,-2\n2025/03/22-07:29:36,129,-5,4083,2,1,-2\n2025/03/23-16:58:49,121,-28,4118,0,1,0\n2025/03/22-16:47:33,117,-26,4108,2,0,2\n2025/03/25-21:57:11,124,-14,4102,2,2,-1\n2025/03/23-11:50:26,105,-6,4111,1,2,2\n2025/03/22-08:00:10,129,-9,4081,2,2,0\n2025/03/25-03:09:00,120,-12,4111,-2,-1,-1\n2025/03/23-12:06:23,102,-10,4090,-1,-2,-2\n2025/03/25-20:23:23,113,-9,4116,0,-2,-2\n2025/03/22-18:21:47,111,-6,4109,-1,-1,0\n2025/03/25-13:59:43,107,-20,4115,1,-2,-1\n2025/03/22-08:22:41,127,-21,4093,-1,0,1\n2025/03/22-05:44:05,115,-16,4086,1,1,2\n2025/03/21-10:23:47,123,-26,4087,0,2,2\n2025/03/22-18:14:10,114,-19,4098,0,-2,-1\n2025/03/21-16:18:06,109,-21,4106,-2,1,1\n2025/03/22-19:32:15,115,-26,4117,-2,2,2\n2025/03/21-20:53:46,111,-25,4090,1,0,-2\n2025/03/21-03:18:52,109,-25,4090,-1,2,-2\n2025/03/21-18:21:08,105,-26,4106,-2,0,-1\n2025/03/23-05:28:49,108,-23,4085,0,-1,2\n2025/03/24-05:40:54,123,-11,4107,-2,-2,2\n2025/03/24-16:26:01,116,-1,4110,-2,0,-2\n2025/03/24-07:36:15,107,-18,4100,-1,0,-2\n2025/03/21-07:10:29,115,-26,4096,-2,0,1\n2025/03/21-01:27:55,129,-27,4102,0,-1,1\n2025/03/23-17:00:35,111,-7,4085,-1,-1,1\n2025/03/23-10:37:43,106,-11,4080,-1,2,-1\n2025/03/21-13:43:05,118,-16,4090,0,-1,0\n2025/03/25-13:14:15,110,-10,4098,-2,0,2\n2025/03/25-08:26:59,110,-24,4097,-2,2,-2\n2025/03/23-13:33:43,128,-7,4109,-1,0,1\n2025/03/21-04:33:47,126,-17,4080,2,0,-1\n2025/03/22-05:22:54,123,-9,4099,0,-1,-2\n2025/03/25-12:00:12,119,-2,4085,-1,2,0\n2025/03/22-16:54:07,112,-1,4103,2,1,-2\n2025/03/24-10:37:13,108,-30,4110,-2,2,1\n2025/03/23-09:18:14,100,-13,4107,-1,-1,2\n2025/03/21-16:40:24,129,-7,4107,-1,1,-2\n2025/03/24-16:17:57,111,-6,4099,-2,2,-2\n2025/03/24-00:00:27,112,-14,4096,1,-2,1\n2025/03/22-10:18:53,107,-30,4088,-1,1,2\n2025/03/22-20:00:31,104,-7,4112,2,-1,-2\n2025/03/23-06:35:43,129,-3,4088,-1,2,0\n2025/03/25-18:25:42,120,-12,4104,2,-1,-2\n2025/03/24-19:13:46,123,-21,4111,-2,1,2\n2025/03/21-04:20:04,101,-29,4097,-1,1,-2\n2025/03/22-03:07:43,126,-18,4104,2,-1,0\n2025/03/21-17:32:36,129,-12,4116,2,-1,-2\n2025/03/23-01:46:44,120,-16,4107,-2,1,2\n2025/03/21-08:55:25,116,-12,4094,-2,0,2\n2025/03/25-02:42:51,101,-23,4112,0,2,0\n2025/03/25-14:02:45,106,-3,4118,-1,1,-2\n2025/03/21-05:38:48,105,-21,4111,-1,-1,-2\n2025/03/22-02:28:24,123,-25,4095,-2,0,1\n2025/03/25-04:59:20,122,-19,4103,-1,-2,2";

int count = 0;


// Sensors ---------------------------------------------------------------------
unsigned long sensorUpdateInterval = 10; // 60 Hz 
unsigned long accelerationPollInterval = 50; // 20 Hz
/* static float maxAccelMag = 0;
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
static float curGyrZ; */
bool useEvents = true; // Toggle events usage
bool bsecDebug = false;
bool uploadBHI260Firmware = false;
//------------------------------------------------------------------------------

// Event memory ----------------------------------------------------------------
int accTreshold = 8000;
int gyrTreshold = 600;
const int eventMemorySize = 80;
static int toSave = 0;
static int16_t eventMemory[6][eventMemorySize];
static String eventTime[eventMemorySize];
static int memoryIndex = 0;
static int eventCounter = 0;
bool eventDetected = false;
bool savingEvent = false;
static String curTime;
static String saveBuffer = "";
const int writeLength = 1; // Multiplies the eventMemorySize to determine how many values to save at once
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

  pinMode(GPIO_1_PIN, OUTPUT);
  pinMode(GPIO_2_PIN, OUTPUT);
  digitalWrite(GPIO_1_PIN, LOW);
  digitalWrite(GPIO_2_PIN, LOW);

  if (uploadBHI260Firmware) {
    // Upload BHI260 firmware if needed
    Serial.println("Uploading BHI260 firmware...");
    bhi260ap_load_firmware();
    Serial.println("BHI260 firmware uploaded successfully.");

    pinMode(sensorCS, OUTPUT);
    digitalWrite(sensorCS, HIGH);
    pinMode(SD_CS_PIN, OUTPUT);
    digitalWrite(SD_CS_PIN, HIGH);
    delay(1000);
  }

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
  digitalWrite(GPIO_1_PIN, HIGH);
  digitalWrite(GPIO_2_PIN, LOW);


  for(int count = 0; count < 1000; count++) {
    static auto readTime = millis();

    // Check sensor values every sensorUpdateInterval milliseconds
    if (millis() - readTime >= sensorUpdateInterval) {
      readTime = millis();
      
      if(readSensors)
      {
        digitalWrite(GPIO_1_PIN, HIGH);
        digitalWrite(GPIO_2_PIN, HIGH);
        sensorUpdate();
        digitalWrite(GPIO_1_PIN, LOW);
        digitalWrite(GPIO_2_PIN, HIGH);
        sensorGetAccX();
        sensorGetAccY();
        sensorGetAccZ();
        sensorGetGyroX();
        sensorGetGyroY();
        sensorGetGyroZ();
        sensorReadMagX();
        sensorReadMagY();
        sensorReadMagZ();
        sensorReadTemperature();
        sensorReadPressure();
        sensorReadVOC();
        sensorReadCO2();
        sensorReadHumidity();
        digitalWrite(GPIO_1_PIN, LOW);
        digitalWrite(GPIO_2_PIN, LOW);
      }
      
      if(readRTC) {
        digitalWrite(GPIO_1_PIN, HIGH);
        digitalWrite(GPIO_2_PIN, HIGH);
        rtcReadTime();
        digitalWrite(GPIO_1_PIN, LOW);
        digitalWrite(GPIO_2_PIN, LOW);
      }

      if (writeToSD){
        digitalWrite(GPIO_1_PIN, HIGH);
        digitalWrite(GPIO_2_PIN, HIGH);
        sdWrite(line, "data.csv");
        digitalWrite(GPIO_1_PIN, LOW);
        digitalWrite(GPIO_2_PIN, LOW);
      }

      if (writeToSDEvent){
        digitalWrite(GPIO_1_PIN, HIGH);
        digitalWrite(GPIO_2_PIN, HIGH);
        sdWrite(lineEvent, "event.csv");
        digitalWrite(GPIO_1_PIN, LOW);
        digitalWrite(GPIO_2_PIN, LOW);
      }
    }
  }
}