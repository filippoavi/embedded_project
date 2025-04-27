#ifndef SENSORS_H
#define SENSORS_H
#define BHY2_RD_WR_LEN          256    /* MCU maximum read write length */

////////////////////////////

#include <Arduino.h>
#include "SPI.h"
#include "objects.h"
#include <stdbool.h>
#include "bhy2.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "bhy2_parse.h"

#include "mbed_wait_api.h"

#include "bhy2_klio.h"
#include "bhy2_swim.h"
#include "bhy2_bsec.h"
#include "bhy2_head_tracker.h"
//#include "coines.h"

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

//#include "Arduino_BHY2.h"

/* extern SensorXYZ accel;
extern SensorXYZ gyro;
extern SensorXYZ mag;
extern SensorQuaternion rotation;
extern Sensor temperature;
extern Sensor gas;
extern Sensor pressure;
extern SensorBSEC bsec; */
extern float dataBuffer[3][6]; // Buffer to store sensor data, 6 sensors, 200 characters

void sensorSetup(struct bhy2_dev &bhy2_device);
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

void sensor_data_callback(const struct bhy2_fifo_parse_data_info *info, void *private_data);
int8_t sensorUpdate(struct bhy2_dev &bhy2_device);

#endif // SENSORS_H