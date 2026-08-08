#pragma once
// Sourced from UniGeek's shipped m5_cardputer_adv board support (confirmed
// accurate for this exact hardware during research) -- verify against your
// physical unit's silkscreen/schematic before first power-on if in doubt.
//
// TCA8418 keyboard pins confirmed 2026-08-08 by reading UniGeek's actual
// board source directly (local checkout: ~/src/unigeek-main/firmware/boards/
// m5_cardputer_adv/pins_arduino.h and core/Keyboard.h), not guessed. That
// source defines:
//   #define KB_INT       11
//   #define KB_I2C_SDA    8
//   #define KB_I2C_SCL    9
//   #define KB_I2C_ADDR 0x34
// and Keyboard.h's begin() calls `Wire1.begin(KB_I2C_SDA, KB_I2C_SCL)` --
// UniGeek dedicates the ESP32-S3's second I2C bus (Wire1) to the TCA8418 +
// ES8311 codec, keeping `Wire` free for the Grove/hat port (GROVE_SDA=2,
// GROVE_SCL=1, used by the CC1101/nRF24 hat's I2C-adjacent signaling and any
// Grove peripheral). This task's Device skeleton (see hal/device.cpp) uses
// the default `Wire` object for simplicity since no Grove I2C peripheral is
// brought up yet; if/when Grove I2C is added, split it onto `Wire1` (or vice
// versa) to match UniGeek's bus separation and avoid the keyboard and a
// Grove I2C device fighting over one bus.
//
// Cross-checked against the M5Stack StampS3 base module's Arduino variant
// (framework-arduinoespressif32/variants/m5stack_stamp_s3/pins_arduino.h):
// GPIO 8/9/11 are plain unreserved GPIOs there (module's own SDA/SCL default
// to 13/15, TX/RX to 43/44), so no conflict with the StampS3 module itself.
#define CP_ADV_KB_I2C_ADDR   0x34
#define CP_ADV_KB_INT_PIN    11
#define CP_ADV_KB_SDA_PIN     8
#define CP_ADV_KB_SCL_PIN     9

#define CP_ADV_SPI_SCK       40
#define CP_ADV_SPI_MISO      39
#define CP_ADV_SPI_MOSI      14

#define CP_ADV_LORA_CS_PIN   5
#define CP_ADV_CC1101_CS_PIN 1
#define CP_ADV_CC1101_GDO0_PIN 2
#define CP_ADV_NRF24_CSN_PIN 1  // shared with CC1101_CS -- electrically exclusive, see Phase 4 spec
#define CP_ADV_NRF24_CE_PIN  2  // shared with CC1101_GDO0

// ─── Display (ST7789, own SPI bus -- distinct from the shared hat/SD bus
// above) ─────────────────────────────────────────────────────────────────
// Also sourced from UniGeek's pins_arduino.h. Not yet wired up by this
// task's Device skeleton (Step 2 stubs display_ready_ = true without a real
// panel init call), but recorded here now since they were confirmed during
// the same research pass and later tasks will need them.
#define CP_ADV_LCD_MOSI      35
#define CP_ADV_LCD_SCLK      36
#define CP_ADV_LCD_CS        37
#define CP_ADV_LCD_DC        34
#define CP_ADV_LCD_RST       33
#define CP_ADV_LCD_BL        38
#define CP_ADV_LCD_WIDTH     135
#define CP_ADV_LCD_HEIGHT    240
