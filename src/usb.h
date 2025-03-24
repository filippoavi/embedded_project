#ifndef USB_H
#define USB_H

#include "mbed.h"
#include "blockdevice/COMPONENT_SD/include/SD/SDBlockDevice.h"
//#include "lib/SDBlockDevice/SDBlockDevice.h"
#include "USBMSD.h"

extern SDBlockDevice sdbd;
extern USBMSD usb;

void usbLoop();

#endif // USB_H