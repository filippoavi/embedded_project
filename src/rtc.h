#ifndef RTC_H
#define RTC_H

#include "RTClib.h"
#include "Arduino.h"

extern RTC_DS3231 rtc;

/* extern Sensor temp(SENSOR_ID_TEMP);
extern Sensor gas(SENSOR_ID_GAS); */

void rtcSetup();
String rtcReadTime();

#endif // RTC_H