#include "sensors.h"

//------------------------------------------------------------------------------
SensorXYZ accel(SENSOR_ID_ACC);
SensorXYZ gyro(SENSOR_ID_GYRO);
SensorXYZ mag(SENSOR_ID_MAG);
SensorQuaternion rotation(SENSOR_ID_RV);
Sensor temperature(SENSOR_ID_TEMP);
Sensor gas(SENSOR_ID_GAS);
Sensor pressure(SENSOR_ID_BARO);
SensorBSEC bsec(SENSOR_ID_BSEC);
const int8_t sensorCS = D6;
//------------------------------------------------------------------------------
void sensorSetup() {
  digitalWrite(sensorCS, LOW);
  BHY2.begin();
  accel.begin();        // Accelerometer
  gyro.begin();         // Gyroscope
  mag.begin();          // Magnetometer
  temperature.begin();  // Temperature
  pressure.begin();     // Pressure
  bsec.begin();         // BME sensor readings
}
//------------------------------------------------------------------------------
void sensorReadSerial() {
  // Standalone Accelerometer values
  Serial.print("acc_X:");
  Serial.print(accel.x());
  Serial.print(",");
  Serial.print("acc_Y:");
  Serial.print(accel.y());
  Serial.print(",");
  Serial.print("acc_Z:");
  Serial.print(accel.z());
  Serial.print(",");
  // Standalone Gyroscope values
  Serial.print("gyro_X:");
  Serial.print(gyro.x());
  Serial.print(",");
  Serial.print("gyro_Y:");
  Serial.print(gyro.y());
  Serial.print(",");
  Serial.print("gyro_Z:");
  Serial.print(gyro.z());
  Serial.print(",");
  // Standalone Magnetometer values
  Serial.print("mag_X:");
  Serial.print(mag.x());
  Serial.print(",");
  Serial.print("mag_Y:");
  Serial.print(mag.y());
  Serial.print(",");
  Serial.print("mag_Z:");
  Serial.print(mag.z());
  Serial.print(",");
  // Standalone Temeprature values
  Serial.print(String("Temperature:") + String(temperature.value()) + String(","));
  // Standalone Pressure values
  Serial.print(String("Pressure:") + String(pressure.value()) + String(","));
  // Standalone BME sensor redings
  Serial.print(String("VOC:") + String(bsec.b_voc_eq()) + String(","));
  Serial.print(String("CO2:") + String(bsec.co2_eq()) + String(","));
  Serial.println(String("Humidity:") + String(bsec.comp_h()));
}
//------------------------------------------------------------------------------
// Define a macro function to generate functions to read sensor values
#define SENSOR_READ_FUNC(name, sensor, value) \
String name() { \
  return String(sensor.value()); \
}
// Functions to read sensor values
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