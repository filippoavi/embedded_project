/* Reference used in this code:
 * https://github.com/arduino-libraries/Arduino_BHY2/tree/main
 */

#include "sensor_manager.h"

//------------------------------------------------------------------------------
SensorXYZ accel(SENSOR_ID_ACC);
SensorXYZ gyro(SENSOR_ID_GYRO);
SensorXYZ mag(SENSOR_ID_MAG);
SensorQuaternion rotation(SENSOR_ID_RV);
Sensor temperature(SENSOR_ID_TEMP);
Sensor gas(SENSOR_ID_GAS);
Sensor pressure(SENSOR_ID_BARO);
SensorBSEC bsec(SENSOR_ID_BSEC);
//------------------------------------------------------------------------------
void sensorSetup() {
  BHY2.begin();
  accel.begin();        // Accelerometer
  gyro.begin();         // Gyroscope
  mag.begin();          // Magnetometer
  temperature.begin();  // Temperature
  pressure.begin();     // Pressure
  bsec.begin();     // BSEC (Gas) Sensor
}
//------------------------------------------------------------------------------

// Define a macro function to generate functions to read sensor values to String
#define SENSOR_READ_FUNC(name, sens, axis) \
String name() { \
  return String(sens.axis()); \
}
// Functions to read sensor values as String
SENSOR_READ_FUNC(sensorReadAccX, accel, x);
SENSOR_READ_FUNC(sensorReadAccY, accel, y);
SENSOR_READ_FUNC(sensorReadAccZ, accel, z);
SENSOR_READ_FUNC(sensorReadGyroX, gyro, x);
SENSOR_READ_FUNC(sensorReadGyroY, gyro, y);
SENSOR_READ_FUNC(sensorReadGyroZ, gyro, z);
SENSOR_READ_FUNC(sensorReadMagX, mag, x);
SENSOR_READ_FUNC(sensorReadMagY, mag, y);
SENSOR_READ_FUNC(sensorReadMagZ, mag, z);
SENSOR_READ_FUNC(sensorReadTemperature, temperature, value);
SENSOR_READ_FUNC(sensorReadPressure, pressure, value);
SENSOR_READ_FUNC(sensorReadVOC, bsec, b_voc_eq);
SENSOR_READ_FUNC(sensorReadCO2, bsec, co2_eq);
SENSOR_READ_FUNC(sensorReadHumidity, bsec, comp_h);
//----------------------------------------------------------------------------
// Define a macro function to generate functions to get sensor values
#define SENSOR_GET_FUNC(name, sens, axis) \
float name() { \
  return float(sens.axis()); \
}
// Functions to get sensor values as float
SENSOR_GET_FUNC(sensorGetAccX, accel, x);
SENSOR_GET_FUNC(sensorGetAccY, accel, y);
SENSOR_GET_FUNC(sensorGetAccZ, accel, z);
SENSOR_GET_FUNC(sensorGetGyroX, gyro, x);
SENSOR_GET_FUNC(sensorGetGyroY, gyro, y);
SENSOR_GET_FUNC(sensorGetGyroZ, gyro, z);
SENSOR_GET_FUNC(sensorGetMagX, mag, x);
SENSOR_GET_FUNC(sensorGetMagY, mag, y);
SENSOR_GET_FUNC(sensorGetMagZ, mag, z);
SENSOR_GET_FUNC(sensorGetTemperature, temperature, value);
SENSOR_GET_FUNC(sensorGetPressure, pressure, value);
SENSOR_GET_FUNC(sensorGetVOC, bsec, b_voc_eq);
SENSOR_GET_FUNC(sensorGetCO2, bsec, co2_eq);
SENSOR_GET_FUNC(sensorGetHumidity, bsec, comp_h);
//----------------------------------------------------------------------------
// Function to update the reading of the BHI260AP 
int8_t sensorUpdate() {
  BHY2.update();
}
