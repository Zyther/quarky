#include "cc1101_hw.h"

#include "../../../boards/tab5/pins_config.h"

#include <RadioLib.h>

namespace Cc1101Hw {

namespace {

// SPI bus is brought up with our own custom M-Bus pins in init() (below)
// BEFORE this Module is used, then passed to RadioLib already-begun via the
// Module(cs, irq, rst, gpio, spi, spiSettings) constructor overload -- the
// one RadioLib's own header documents as "Will not attempt SPI interface
// initialization", deliberately chosen over the simpler 4-arg constructor
// (which calls SPI.begin() itself, with no way to pass our non-default
// M-Bus pins). rst=RADIOLIB_NC: this module has no dedicated hardware reset
// line on the M-Bus connector (confirmed from both vendor PDFs' pin
// tables -- only MOSI/MISO/SCK/CSN/GDO0/GDO2/GND/5V/HPWR appear); RadioLib
// falls back to its own software reset sequence over SPI in this case, the
// same real chip capability the prior driver's SRES strobe also used.
Module s_module(TAB5_CC1101_CSN_GPIO, TAB5_CC1101_GDO0_GPIO, RADIOLIB_NC,
                 TAB5_CC1101_GDO2_GPIO, SPI);
CC1101 s_radio(&s_module);

bool s_present = false;
float s_frequency_mhz = 433.92f; // RadioLib's own CC1101 default (RADIOLIB_CC1101_DEFAULT_FREQ)

} // namespace

bool init() {
    SPI.begin(TAB5_CC1101_SCK_GPIO, TAB5_CC1101_MISO_GPIO,
              TAB5_CC1101_MOSI_GPIO, TAB5_CC1101_CSN_GPIO);
    int16_t state = s_radio.begin(s_frequency_mhz);
    s_present = (state == RADIOLIB_ERR_NONE);
    return s_present;
}

bool is_present() {
    return s_present;
}

void set_frequency_mhz(float mhz) {
    s_frequency_mhz = mhz;
    s_radio.setFrequency(mhz);
}

float frequency_mhz() {
    return s_frequency_mhz;
}

float rssi() {
    return s_radio.getRSSI();
}

} // namespace Cc1101Hw
