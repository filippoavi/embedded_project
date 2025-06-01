#ifndef DEFINES_H
#define DEFINES_H

#include "Arduino.h"

// SD SPI Chip Select pin
//const uint8_t SD_CS_PIN = 6; // nicla
const uint8_t SD_CS_PIN = D7;

// Sensors SPI Chip Select pin
//const uint8_t sensorCS = 31; // nicla
const uint8_t sensorCS = D6;

// Common pins definitions
extern PinName c_int;
extern PinName c_mosi;
extern PinName c_cs_sens;
extern PinName c_miso;
extern PinName c_clk;

const int spi_freq = 1000000;

#endif // DEFINES_H