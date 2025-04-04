#ifndef USB_H
#define USB_H

#include "mbed.h"
//#include "BlockDevice.h"
//#include "SDBlockDevice.h" //<---- This does not import correctly I don't know why... its in the same place as QSPIFBlockDevice.h and MBRBlockDevice.h that both work fine so ??????? I don't know...........
#include "C:/Users/Filippo/.platformio/packages/framework-arduino-mbed@4.1.5/cores/arduino/mbed/storage/blockdevice/COMPONENT_SD/include/SD/SDBlockDevice.h"
#include "lib/USBMSD_SD/USBMSD_SD.h"
#include "USBMSD.h"

/* #include "PluggableUSBMSD.h"
#include "QSPIFBlockDevice.h"
#include "MBRBlockDevice.h"
#include "FATFileSystem.h" */

void usbLoop();

#endif // USB_H