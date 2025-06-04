#include "usb.h"
#include "defines.h"

uint64_t hz = 16000000;
bool crc_on = false;

SDBlockDevice sdbd(digitalPinToPinName(D8), digitalPinToPinName(D10), digitalPinToPinName(D9), digitalPinToPinName(D7), spi_freq, false);
USBMSD usb(&sdbd);

void usbSetup()
{
    sdbd.init();    // Make sure SD is initialized
    usb.connect();  // If available, start the USB device
}

void usbLoop()
{
    usb.process();
}