#ifndef SENSORS_H
#define SENSORS_H

#include "defines.h"

#include "Arduino.h"
#include "mbed.h"
#include "objects.h"
#include <stdbool.h>
#include "bhy2.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "bhy2_parse.h" 

#include "bhy2_klio.h"
#include "bhy2_swim.h"
#include "bhy2_bsec.h"
#include "bhy2_head_tracker.h"

#include "SensorManager.h"
#include "SensorClass.h"
#include "Sensor.h"
#include "SensorXYZ.h"
#include "SensorBSEC.h"
#include "common.h"
#include "BoschParser.h"

//------------------------------------------------------------------------------

#define BHY2_RD_WR_LEN 256  // MCU maximum read write length
#define SENSOR_DATA_FIXED_LENGTH (10)
#define SENSOR_LONG_DATA_FIXED_LENGTH (21)
//#define WORK_BUFFER_SIZE 1024

const uint8_t BLE_SENSOR_EVT_BATCH_CNT_MAX = (244 / (SENSOR_DATA_FIXED_LENGTH + 2))/10*10;
//#define SENSOR_QUEUE_SIZE   (BLE_SENSOR_EVT_BATCH_CNT_MAX + 20)
#define LONG_SENSOR_QUEUE_SIZE 4

using namespace mbed;

extern mbed::CircularBuffer<SensorDataPacket, SENSOR_QUEUE_SIZE, uint8_t> _sensorQueue;
extern mbed::CircularBuffer<SensorLongDataPacket, LONG_SENSOR_QUEUE_SIZE, uint8_t> _longSensorQueue;
extern uint8_t _sensorsPresent[32];

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

//void parseMetaEvent(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
//void parseDebugMessage(const struct bhy2_fifo_parse_data_info *callback_info, void *callback_ref);
void parseData(const struct bhy2_fifo_parse_data_info *fifoData, void *arg);

void sensor_data_callback(const struct bhy2_fifo_parse_data_info *info, void *private_data);
void convertTime(uint64_t time_ticks, uint32_t *s, uint32_t *ns);
void addSensorData(SensorDataPacket &sensorData);
void addLongSensorData(SensorLongDataPacket &sensorData);
//void dataBufferPrint();
void spiTest();
void printSensors();

extern struct bhy2_dev _bhy2;

#endif // SENSORS_H