#include "touch_gt911.h"
#include "../../boards/tab5/pins_config.h"
#include <Wire.h>

// GT911 I2C address/pins and register map: pulled from the espp/m5stack-tab5
// BSP (same source as Task 5's display init) -- see boards/tab5/pins_config.h
// for the full citation trail (address 0x14, register 0x814E status /
// 0x814F first point, sync-clear write, etc). This is a hand-rolled register
// client rather than a third-party Arduino_GT911 library: the BSP's own
// espp::Gt911 driver documents the exact protocol it uses (itself sourced
// from Espressif's esp-bsp esp_lcd_touch_gt911.c), so transcribing that
// directly is higher-fidelity than pulling in a generic library and hoping
// its register map matches this chip variant/address.

namespace {

bool i2cReadRegister(uint8_t addr, uint16_t reg, uint8_t *buf, size_t len) {
    Wire.beginTransmission(addr);
    Wire.write(static_cast<uint8_t>(reg >> 8));   // register address is big-endian
    Wire.write(static_cast<uint8_t>(reg & 0xFF)); // (2 bytes, MSB first) on GT911
    if (Wire.endTransmission(false) != 0) { // repeated start, keep bus held
        return false;
    }
    size_t got = Wire.requestFrom(static_cast<int>(addr), static_cast<int>(len));
    if (got != len) {
        return false;
    }
    for (size_t i = 0; i < len; i++) {
        buf[i] = Wire.read();
    }
    return true;
}

bool i2cWriteRegisterU8(uint8_t addr, uint16_t reg, uint8_t value) {
    Wire.beginTransmission(addr);
    Wire.write(static_cast<uint8_t>(reg >> 8));
    Wire.write(static_cast<uint8_t>(reg & 0xFF));
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

} // namespace

void TouchGT911::init() {
    // SDA/SCL are the BSP's internal I2C bus pins, not the default board
    // pins -- see TAB5_INTERNAL_I2C_SDA_GPIO/SCL_GPIO in pins_config.h. This
    // is the same bus the display's reset IO-expander lives on.
    //
    // NOTE: the BSP runs this bus at 1 MHz (m5stack-tab5.hpp
    // internal_i2c_clock_speed); Wire.begin() below leaves Arduino's default
    // (100 kHz) in effect. Not raised to match here -- the internal bus is
    // shared with the IMU/RTC/audio codecs the BSP also drives at 1 MHz, and
    // matching that exactly would need those other devices accounted for
    // too. 100 kHz is safely within the GT911's own supported range, so this
    // is a conservative choice, not a correctness bug.
    Wire.begin(TAB5_INTERNAL_I2C_SDA_GPIO, TAB5_INTERNAL_I2C_SCL_GPIO);

    // TODO: drive the GT911 hardware reset pulse via the PI4IOE5V6408
    // IO-expander (TAB5_TOUCH_RST_IOEXP_I2C_ADDR/BIT in pins_config.h)
    // before first use, matching the BSP's assert/10ms/release/50ms
    // sequence. Not implemented here -- see pins_config.h TODO for why
    // (the IO-expander's own register protocol wasn't transcribed in this
    // research pass, same scope cut Task 5 made for the display's reset).
    // The GT911 generally still answers I2C on its power-on-reset default
    // without this pulse, so omitting it is a soft-fail, not a hard-fail.
}

void TouchGT911::read(int16_t &x, int16_t &y, bool &pressed) {
    // Read GT911's touch status + coordinate registers over I2C per the
    // register map documented in pins_config.h (sourced from the espp BSP's
    // Gt911 driver, itself citing Espressif's esp-bsp esp_lcd_touch_gt911.c).
    pressed = false;
    x = 0;
    y = 0;

    uint8_t status = 0;
    if (!i2cReadRegister(TAB5_TOUCH_I2C_ADDR, TAB5_TOUCH_REG_STATUS, &status, 1)) {
        return; // I2C error (e.g. touch not present / not reset) -- no touch
    }

    if ((status & 0x80) == 0) {
        // Bit 7 clear: no new data ready. Nothing to clear/sync either.
        return;
    }

    if ((status & 0x10) == 0x10) {
        // Only the home key was pressed; no coordinate data this cycle.
        // (Home button state is not exposed by the ITouch interface.)
        i2cWriteRegisterU8(TAB5_TOUCH_I2C_ADDR, TAB5_TOUCH_REG_STATUS, 0x00);
        return;
    }

    uint8_t num_points = status & 0x0F;
    if (num_points == 0) {
        i2cWriteRegisterU8(TAB5_TOUCH_I2C_ADDR, TAB5_TOUCH_REG_STATUS, 0x00);
        return;
    }

    // First contact only -- ITouch is a single-point interface (LVGL's
    // pointer indev only needs one point). Layout per contact record:
    // {uint8_t track_id; uint16_t x; uint16_t y; uint16_t area; uint8_t
    // reserved;} = 8 bytes, x/y little-endian on the wire.
    uint8_t point[TAB5_TOUCH_CONTACT_SIZE] = {0};
    if (!i2cReadRegister(TAB5_TOUCH_I2C_ADDR, TAB5_TOUCH_REG_POINT_1, point,
                          sizeof(point))) {
        return;
    }

    uint16_t raw_x = static_cast<uint16_t>(point[1]) | (static_cast<uint16_t>(point[2]) << 8);
    uint16_t raw_y = static_cast<uint16_t>(point[3]) | (static_cast<uint16_t>(point[4]) << 8);
    x = static_cast<int16_t>(raw_x);
    y = static_cast<int16_t>(raw_y);
    pressed = true;

    // Sync/clear signal so the GT911 knows the host consumed this report.
    i2cWriteRegisterU8(TAB5_TOUCH_I2C_ADDR, TAB5_TOUCH_REG_STATUS, 0x00);
}
