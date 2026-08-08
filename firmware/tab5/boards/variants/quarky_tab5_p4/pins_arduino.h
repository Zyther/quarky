#ifndef Pins_Arduino_h
#define Pins_Arduino_h

// -----------------------------------------------------------------------------
// M5Stack Tab5 (ESP32-P4) Arduino variant -- in-repo.
// -----------------------------------------------------------------------------
// This is a MINIMAL FORK of the pioarduino framework's
// variants/esp32p4/pins_arduino.h (the generic ESP32-P4-Function-EV-Board
// variant this project used previously, via `board = esp32-p4-evboard`).
//
// EXACTLY ONE THING IS CHANGED: the seven BOARD_SDIO_ESP_HOSTED_* macros at
// the bottom, which define the SDIO bus wiring between the ESP32-P4 host and
// the Tab5's onboard ESP32-C6 radio co-processor. Everything else -- ETH,
// SDMMC slot/power, the on-chip LDO auto-enable range, the analog/touch/SPI/
// I2C aliases -- is carried forward byte-for-byte so that no already-working
// code path (Task 5 display, Task 6 touch, Task 10 SD, USB CDC) changes
// behaviour. Diff this file against
//   ~/.platformio/packages/framework-arduinoespressif32/variants/esp32p4/pins_arduino.h
// to confirm: the only differences are this comment block and the C6 pins.
//
// WHY: cores/esp32/esp32-hal-hosted.c initialises its static sdio_pin_config_t
// straight from these macros at compile time, and they are unconditional
// #defines with no #ifndef guard -- so a `-D` build flag in platformio.ini
// cannot win against them (the header's definition is seen later and takes
// effect). A variant fork is the only reliable, in-repo, reproducible fix.
//
// The Tab5 values below are corroborated by two independent sources that agree
// exactly:
//   1. The M5Stack Tab5 BSP in esp-cpp/espp
//      (components/m5stack-tab5/include/m5stack-tab5.hpp), where the C6 link
//      nets are labelled SDIO2_*: clk=12, cmd=13, d0=11, d1=10, d2=9, d3=8,
//      reset=15. Recorded with citations in boards/tab5/pins_config.h as
//      TAB5_C6_SDIO_* by Task 10.
//   2. The pioarduino framework's OWN m5stack_tab5 variant
//      (framework-arduinoespressif32/variants/m5stack_tab5/pins_arduino.h),
//      which declares the identical seven values. That variant is not used
//      wholesale here because it also drops BOARD_SDMMC_POWER_*,
//      BOARD_PERIMAN_IO_LDO_* and moves SDA/SCL -- unrelated changes to
//      working code paths. See boards/quarky_tab5_p4.json.
// -----------------------------------------------------------------------------

#include <stdint.h>
#include "soc/soc_caps.h"

// BOOT_MODE 35
// BOOT_MODE2 36 pullup

static const uint8_t TX = 37;
static const uint8_t RX = 38;

// SDA/SCL moved off the eval-board defaults (7/8) to Tab5's actual external
// I2C bus pins (matching the framework's own m5stack_tab5 variant) because
// GPIO 8 is now BOARD_SDIO_ESP_HOSTED_D3 -- a bare Wire.begin() with no pins
// falls back to these, and 7/8 would have handed the C6's live SDIO data
// line to any future Wire/Wire1 caller that doesn't specify pins explicitly.
// This project's own I2C code always calls Wire.begin() with explicit pins
// (TAB5_INTERNAL_I2C_SDA_GPIO/SCL_GPIO = 31/32), so this default is a safety
// net for third-party libraries, not something this project's own code relies on.
static const uint8_t SDA = 53;
static const uint8_t SCL = 54;

// GPIO 36 and below: no extra on-chip LDO needed for the IO bank.
// GPIO 39-48: LDO VO4 (channel 4). GPIO 37-38 (UART) are outside that VO4 range.
static const uint8_t SS = 26;
static const uint8_t MOSI = 32;
static const uint8_t MISO = 33;
static const uint8_t SCK = 36;

static const uint8_t A0 = 16;
static const uint8_t A1 = 17;
static const uint8_t A2 = 18;
static const uint8_t A3 = 19;
static const uint8_t A4 = 20;
static const uint8_t A5 = 21;
static const uint8_t A6 = 22;
static const uint8_t A7 = 23;
static const uint8_t A8 = 49;
static const uint8_t A9 = 50;
static const uint8_t A10 = 51;
static const uint8_t A11 = 52;
static const uint8_t A12 = 53;
static const uint8_t A13 = 54;

static const uint8_t T0 = 2;
static const uint8_t T1 = 3;
static const uint8_t T2 = 4;
static const uint8_t T3 = 5;
static const uint8_t T4 = 6;
static const uint8_t T5 = 7;
static const uint8_t T6 = 8;
static const uint8_t T7 = 9;
static const uint8_t T8 = 10;
static const uint8_t T9 = 11;
static const uint8_t T10 = 12;
static const uint8_t T11 = 13;
static const uint8_t T12 = 14;
static const uint8_t T13 = 15;

//ETH
#define ETH_PHY_TYPE    ETH_PHY_TLK110
#define ETH_PHY_ADDR    1
#define ETH_PHY_MDC     31
#define ETH_PHY_MDIO    52
#define ETH_PHY_POWER   51
#define ETH_RMII_TX_EN  49
#define ETH_RMII_TX0    34
#define ETH_RMII_TX1    35
#define ETH_RMII_RX0    29
#define ETH_RMII_RX1_EN 30
#define ETH_RMII_CRS_DV 28
#define ETH_RMII_CLK    50
#define ETH_CLK_MODE    EMAC_CLK_EXT_IN

//SDMMC
// Slot 0 on the ESP32-P4 is IOMUX-fixed in silicon (clk=43, cmd=44,
// d0..d3=39..42), which matches the Tab5's real microSD wiring -- see
// boards/tab5/pins_config.h TAB5_SD_* and task-10-report.md caveat (c).
// BOARD_SDMMC_POWER_* is carried over unchanged from the EV-board variant:
// task-10-report.md caveat (d) flagged it as genuinely unverified for Tab5,
// and this hotfix deliberately does not change unverified behaviour it was
// not scoped to fix. If SD mount ever proves to fail on hardware because of
// it, dropping both POWER macros (as the framework's m5stack_tab5 variant
// does) is the first thing to try.
#define BOARD_HAS_SDMMC
#define BOARD_SDMMC_SLOT           0
#define BOARD_SDMMC_POWER_CHANNEL  4
#define BOARD_SDMMC_POWER_PIN      45
#define BOARD_SDMMC_POWER_ON_LEVEL LOW

// On-chip GP LDO: periman enables VO4 when a GPIO in the range is used (see esp32-hal-ldo.c).
#define BOARD_PERIMAN_IO_LDO_AUTO        1
#define BOARD_PERIMAN_IO_LDO0_CHANNEL    4   // LDO_VO4 on ESP32-P4
#define BOARD_PERIMAN_IO_LDO0_GPIO_MIN   39  // GPIO 39-48 on VO4
#define BOARD_PERIMAN_IO_LDO0_GPIO_MAX   48
#define BOARD_PERIMAN_IO_LDO0_VOLTAGE_MV 3300

//WIFI/BLE - ESP32C6 co-processor over SDIO (slot 1)
// *** THE ONLY FUNCTIONAL CHANGE vs. variants/esp32p4/pins_arduino.h ***
// EV-board (wrong for Tab5): clk=18 cmd=19 d0=14 d1=15 d2=16 d3=17 reset=54
// Tab5 (correct, per both sources cited in the header comment above):
#define BOARD_HAS_SDIO_ESP_HOSTED
#define BOARD_SDIO_ESP_HOSTED_CLK   12
#define BOARD_SDIO_ESP_HOSTED_CMD   13
#define BOARD_SDIO_ESP_HOSTED_D0    11
#define BOARD_SDIO_ESP_HOSTED_D1    10
#define BOARD_SDIO_ESP_HOSTED_D2    9
#define BOARD_SDIO_ESP_HOSTED_D3    8
#define BOARD_SDIO_ESP_HOSTED_RESET 15

#endif /* Pins_Arduino_h */
