#ifndef SENSORS_H
#define SENSORS_H

//1
/* #define SPI_PSELMOSI0 SPI_MOSI
#define SPI_PSELMISO0 SPI_MISO
#define SPI_PSELSCK0 SPI_SCK
#define CS_FLASH D6 */
//3
/* #define P0_10 D5
#define p24 D4
#define PIN_ESLOV_INT D3
#define INT_BHI260 D6 */

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