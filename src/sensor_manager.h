#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#include "defines.h"
#include "Arduino.h"
#include "Arduino_BHY2.h"

extern SensorXYZ accel;
extern SensorXYZ gyro;
extern SensorXYZ mag;
extern SensorQuaternion rotation;
extern Sensor temperature;
extern Sensor gas;
extern Sensor pressure;
extern SensorBSEC bsec;

//------------------------------------------------------------------------------

void sensorSetup();
int8_t sensorUpdate();

String sensorReadAccX();
String sensorReadAccY();
String sensorReadAccZ();
String sensorReadGyroX();
String sensorReadGyroY();
String sensorReadGyroZ();
String sensorReadMagX();
String sensorReadMagY();
String sensorReadMagZ();
String sensorReadTemperature();
String sensorReadPressure();
String sensorReadVOC();
String sensorReadCO2();
String sensorReadHumidity();

float sensorGetAccX();
float sensorGetAccY();
float sensorGetAccZ();
float sensorGetGyroX();
float sensorGetGyroY();
float sensorGetGyroZ();
float sensorGetMagX();
float sensorGetMagY();
float sensorGetMagZ();
float sensorGetTemperature();
float sensorGetPressure();
float sensorGetVOC();
float sensorGetCO2();
float sensorGetHumidity();

#endif // SENSOR_MANAGER_H