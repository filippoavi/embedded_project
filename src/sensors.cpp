#include "sensors.h"

//------------------------------------------------------------------------------
SensorXYZ accel(SENSOR_ID_ACC);
SensorXYZ gyro(SENSOR_ID_GYRO);
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
  temperature.begin();  // Temperature
  pressure.begin();     // Pressure
  bsec.begin();         // BME sensor readings
}
//------------------------------------------------------------------------------
void sensorRead() {
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

  // Standalone Temeprature values
  Serial.print(String("Temperature:") + String(temperature.value()) + String(","));

  // Standalone Pressure values
  Serial.print(String("Pressure:") + String(pressure.value()) + String(","));

  // Standalone BME sensor redings
  Serial.print(String("VOC:") + String(bsec.b_voc_eq()) + String(","));
  Serial.print(String("CO2:") + String(bsec.co2_eq()) + String(","));
  Serial.println(String("Humidity:") + String(bsec.comp_h()));
}