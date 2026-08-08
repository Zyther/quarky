#pragma once
#include <stdint.h>

// Minimal driver for the Tab5's two PI4IOE5V6408 I2C IO-expanders.
//
// Several things on this board that look like GPIOs are not reachable from the
// ESP32-P4 at all -- they hang off one of two PI4IOE5V6408 expanders on the
// internal I2C bus (TAB5_INTERNAL_I2C_SDA_GPIO/SCL_GPIO):
//   * 0x43 -- LCD_RST (P4), TP_RST (P5), CAM_RST (P6), SPK_EN (P1)
//   * 0x44 -- WLAN_PWR_EN (P0, the ESP32-C6's power rail), USB_5V_EN, charger
// Register map and bit assignments are in boards/tab5/pins_config.h, sourced
// from the espp/m5stack-tab5 BSP and espp's pi4ioe5v driver.
//
// Task 5 and Task 6 both left "drive the reset line via the IO-expander" as a
// TODO because this register protocol had not been transcribed; the C6 SDIO
// hotfix needed it for WLAN_PWR_EN, so it lives here now and both users share
// it rather than each open-coding I2C register pokes.
namespace tab5_ioexp {

// Drive one expander pin to `level` as a push-pull output: sets the output
// latch, takes the pin out of high-Z, and switches its direction to output --
// in that order, so the pin is never briefly driven to the wrong level.
//
// Every register access is read-modify-write on the single bit requested.
// Deliberately NOT the BSP's full expander init (software reset + rewrite of
// every register): these expanders also gate USB 5V and the battery charger,
// and blind-resetting them from application code could disturb those.
//
// The caller is responsible for having begun Wire on the internal bus.
// Returns false if any I2C transaction fails.
bool set_output(uint8_t i2c_addr, uint8_t bit, bool level);

}  // namespace tab5_ioexp
