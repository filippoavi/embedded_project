#include "usb.h"

SDBlockDevice sd(PTE3, PTE1, PTE2, PTE4);
USBMSD usb(&sd);

int main() {

    while(true) {
        usb.process();
    }

    return 0;
}