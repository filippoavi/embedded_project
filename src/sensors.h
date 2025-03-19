#ifndef SENSORS_H
#define SENSORS_H

#include "Arduino_BHY2.h"

extern SensorXYZ accel;
extern SensorXYZ gyro;
extern SensorXYZ mag;
extern SensorQuaternion rotation;
extern Sensor temperature;
extern Sensor gas;
extern Sensor pressure;
extern SensorBSEC bsec;

void sensorSetup();
void sensorReadSerial();
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

#endif // SENSORS_H