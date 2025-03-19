#include "Nicla_System.h"     // LED usage
//#include "ArduinoLowPower.h"  // Power management (not working for this board)
#include "sd_card.h"
#include "sensors.h"
#define DISABLE_FS_H_WARNING  // Disable warning for type File not defined.

// SD card ---------------------------------------------------------------------
bool sdAdvancedCheck = true;
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
  sensorSetup();
  sdSetup();
  // Signal startup completed through LED blink
  nicla::leds.setColor(green);
  delay(1000);
  nicla::leds.setColor(off);
  if (sdAdvancedCheck) {
    sdCheck();
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
    sensorRead();
  }
}