#include "Arduino_BHY2.h"

#include "BoschSensortec.h"
#include "BoschParser.h"
#include "Wire.h"

#include "mbed.h"

Arduino_BHY2::Arduino_BHY2() :
  _debug(NULL),
  _pingTime(0),
  _timeout(120000),
  _startTime(0),
/*   _eslovIntPin(PIN_ESLOV_INT), */
  _niclaConfig(NICLA_BLE_AND_I2C)
{
}

Arduino_BHY2::~Arduino_BHY2()
{
}

void Arduino_BHY2::pingI2C() {
  int currTime = millis();
  if ((currTime - _pingTime) > 30000) {
    _pingTime = currTime;
#ifdef USE_FASTCHG_TO_KICK_WATCHDOG
    //Read charger reg
    nicla::checkChgReg();
#else
    //Read LDO reg
#endif
  }
}

void Arduino_BHY2::setLDOTimeout(int time) {
  _timeout = time;
}

bool Arduino_BHY2::begin(NiclaConfig config, NiclaWiring niclaConnection)
{
  bool res;
  _niclaConfig = config;

  _startTime = millis();
  _pingTime = millis();
  res = sensortec.begin() & res;

  sensortec.bsecSetBoardTempOffset(0.5f);//assuming the device is powered by USB, if on battery only, use a negative value such as -3.0

  if (!(_niclaConfig & NICLA_STANDALONE)) {
    if (_niclaConfig & NICLA_I2C) {
      //Start Eslov Handler
/*       pinMode(_eslovIntPin, OUTPUT); */
    }
  }

  if (_debug) {
/*     _debug->print("Eslov int pin: ");
    _debug->println(_eslovIntPin); */
  }

  return res;
}

bool Arduino_BHY2::begin(NiclaSettings& settings)
{
  uint8_t niclaSettings = settings.getConfiguration();
  if (niclaSettings & NICLA_STANDALONE) {
     _niclaConfig = NICLA_STANDALONE;
  } else if ((niclaSettings & NICLA_I2C) && (niclaSettings & NICLA_BLE)) {
    _niclaConfig = NICLA_BLE_AND_I2C;
  }
  else if (niclaSettings & NICLA_I2C) {
    _niclaConfig = NICLA_I2C;
  }
  else {
    _niclaConfig = NICLA_BLE;
  }
  if (niclaSettings & NICLA_AS_SHIELD) {
    return begin(_niclaConfig, NICLA_AS_SHIELD);
  } else {
    return begin(_niclaConfig, NICLA_VIA_ESLOV);
  }
}

void Arduino_BHY2::update()
{
  pingI2C();

  sensortec.update();
}

// Update and then sleep
void Arduino_BHY2::update(unsigned long ms)
{
  update();
  delay(ms);
}

/* void Arduino_BHY2::delay(unsigned long ms)
{
  unsigned long start = millis();
  unsigned long elapsed = 0;
  if (_niclaConfig & NICLA_BLE) {
    while (elapsed < ms) {
      elapsed = millis() - start;
    }
  }
} */

void Arduino_BHY2::configureSensor(SensorConfigurationPacket& config)
{
  sensortec.configureSensor(config);
}

void Arduino_BHY2::configureSensor(uint8_t sensorId, float sampleRate, uint32_t latency)
{
  SensorConfigurationPacket config;
  config.sensorId = sensorId;
  config.sampleRate = sampleRate;
  config.latency = latency;
  sensortec.configureSensor(config);
}

void Arduino_BHY2::addSensorData(SensorDataPacket &sensorData)
{
  sensortec.addSensorData(sensorData);
}

void Arduino_BHY2::addLongSensorData(SensorLongDataPacket &sensorData)
{
  sensortec.addLongSensorData(sensorData);
}

uint8_t Arduino_BHY2::availableSensorData()
{
  return sensortec.availableSensorData();
}

uint8_t Arduino_BHY2::availableLongSensorData()
{
  return sensortec.availableLongSensorData();
}

bool Arduino_BHY2::readSensorData(SensorDataPacket &data)
{
  return sensortec.readSensorData(data);
}

bool Arduino_BHY2::readLongSensorData(SensorLongDataPacket &data)
{
  return sensortec.readLongSensorData(data);
}

bool Arduino_BHY2::hasSensor(uint8_t sensorId)
{
  return sensortec.hasSensor(sensorId);
}

void Arduino_BHY2::parse(SensorDataPacket& data, DataXYZ& vector)
{
  DataParser::parse3DVector(data, vector);
}

void Arduino_BHY2::parse(SensorDataPacket& data, DataOrientation& vector)
{
  DataParser::parseEuler(data, vector);
}

void Arduino_BHY2::parse(SensorDataPacket& data, DataOrientation& vector, float scaleFactor)
{
  DataParser::parseEuler(data, vector, scaleFactor);
}

void Arduino_BHY2::debug(Stream &stream)
{
  _debug = &stream;
  if (_niclaConfig & NICLA_I2C) {
  }
  if (_niclaConfig & NICLA_BLE) {
  }
  sensortec.debug(stream);
  BoschParser::debug(stream);
}

Arduino_BHY2 BHY2;
