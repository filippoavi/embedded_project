// Date and time functions using a DS3231 RTC connected via I2C and Wire lib
#include "rtc.h"

RTC_DS3231 rtc;

Sensor temp(SENSOR_ID_TEMP);
Sensor gas(SENSOR_ID_GAS);

void setup () {

  Serial.begin(57600);

  BHY2.begin();
  temp.begin();
  gas.begin();

#ifndef ESP8266
  while (!Serial); // wait for serial port to connect. Needed for native USB
#endif

  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (rtc.lostPower()) {
    Serial.println("RTC lost power, let's set the time!");
    // When time needs to be set on a new device, or after a power loss, the
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    // This line sets the RTC with an explicit date & time, for example to set
    // January 21, 2014 at 3am you would call:
    // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
  }

  // When time needs to be re-set on a previously configured device, the
  // following line sets the RTC to the date & time this sketch was compiled
  // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // This line sets the RTC with an explicit date & time, for example to set
  // January 21, 2014 at 3am you would call:
  // rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
}


unsigned long counter = 0;

DateTime now;

void loop () {

    BHY2.update();
    now = rtc.now();

    Serial.print(now.year(), DEC);
    Serial.print('/');
    Serial.print(now.month(), DEC);
    Serial.print('/');
    Serial.print(now.day(), DEC);
    Serial.print(' ');
    Serial.print(now.hour(), DEC);
    Serial.print(':');
    Serial.print(now.minute(), DEC);
    Serial.print(':');
    Serial.print(now.second(), DEC);

    Serial.print(" RTC Temperature: ");
    Serial.print(rtc.getTemperature());
    Serial.print(" C");

    Serial.print(String(" BHY - Temperature: ") + String(temp.value(),3));
    Serial.print(' ');
    Serial.print(String("Gas: ") + String(gas.value(),3));
    Serial.println();

    delay(1000);
}
