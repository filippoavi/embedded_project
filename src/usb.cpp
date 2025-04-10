#include "usb.h"

uint64_t hz = 16000000;
bool crc_on = false;

SDBlockDevice sdbd(PD_8, PD_10, PD_9, PD_7, hz, crc_on);
USBMSD usb(&sdbd);

void usbLoop()
{
    while (true) {
        usb.process();
    }
}