#include "usb.h"

uint64_t hz = 16000000;
bool crc_on = false;

SDBlockDevice sdbd(P0_27, P0_28, P0_11, P0_29, hz, crc_on);
USBMSD usb(&sdbd);

void usbLoop()
{
    while (true) {
        usb.process();
    }
}