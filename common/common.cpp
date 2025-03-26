#include <Arduino.h>

#include <common.h>

namespace mw {

void hang(uint16_t ms) {
    const unsigned long end_t = millis() + ms;
    while (millis() < end_t);
}

}