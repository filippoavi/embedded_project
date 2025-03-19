#ifndef SENSORS_H
#define SENSORS_H

#include "Arduino_BHY2.h"

extern SensorXYZ accel;
extern SensorXYZ gyro;
extern SensorQuaternion rotation;
extern Sensor temperature;
extern Sensor gas;
extern Sensor pressure;
extern SensorBSEC bsec;

void sensorSetup();
void sensorRead();

#endif // SENSORS_H