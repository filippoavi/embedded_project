#ifndef SD_CARD_H
#define SD_CARD_H

#include "SdFat.h"
#include "sdios.h"
#include "SPI.h"

extern SdFs sd;
extern SdFile myFile;
extern cid_t cid;
extern csd_t csd;
extern scr_t scr;
extern uint8_t cmd6Data[64];
extern uint32_t eraseSize;
extern uint32_t ocr;

void sdSetup();
void sdCheck();
void sdTestWrite();
void sdWrite(String line);

#endif // SD_CARD_H