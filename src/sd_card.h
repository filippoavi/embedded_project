#ifndef SD_CARD_H
#define SD_CARD_H

#include "SdFat.h"
#include "sdios.h"
#include "SPI.h"

extern SdFs sd;
extern cid_t cid;
extern csd_t csd;
extern scr_t scr;
extern uint8_t cmd6Data[64];
extern uint32_t eraseSize;
extern uint32_t ocr;

void hexDmp(void* reg, uint8_t size);
void cidDmp();
void clearSerialInput();
void csdDmp();
void errorPrint();
bool mbrDmp();
void dmpVol();
void printCardType();
void printConfig(SdSpiConfig config);
void printConfig(SdioConfig config);
void sdSetup();
void sdCheck();

#endif // SD_CARD_H