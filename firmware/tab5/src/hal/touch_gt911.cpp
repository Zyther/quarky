#include "touch_gt911.h"
#include "io_expander.h"
#include "../../boards/tab5/pins_config.h"
#include <Arduino.h>
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

// Latched availability. Once the GT911 has proven unreachable we stop talking
// to it entirely. Without this, a touch panel that does not answer produces two
// framework error lines per read() -- i.e. ~30 lines/second forever at the
// 5ms loop() cadence -- which floods the serial console, drowns every other
// diagnostic, and burns bus time on a device that is not there. Observed
// exactly that on real hardware once the C6 fix let the board reach loop() for
// the first time. Degrade quietly instead: log once, then no-op.
bool s_available = false;
uint8_t s_consecutive_failures = 0;
constexpr uint8_t kMaxConsecutiveFailures = 10;

// The GT911 answers at one of two addresses depending on the level sampled on
// its INT pin as reset is released: 0x14 or 0x5D. The BSP configures 0x14 (see
// TAB5_TOUCH_I2C_ADDR) and init() drives INT high to select it, but the strap
// is timing-sensitive and INT may also be loaded by other circuitry, so the
// panel can still come up on the alternate address. Probe both and remember
// which one answered rather than hard-failing on the expected one.
constexpr uint8_t kAltTouchI2CAddr = 0x5D;
uint8_t s_addr = TAB5_TOUCH_I2C_ADDR;

// Read-only presence check: address the device and see whether anything ACKs.
bool i2cProbe(uint8_t addr) {
    Wire.beginTransmission(addr);
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

    // GT911 hardware reset pulse, via the PI4IOE5V6408 IO-expander at 0x43
    // (TAB5_TOUCH_RST_IOEXP_I2C_ADDR/BIT) -- there is no raw reset GPIO on
    // this board. Task 6 left this as a TODO because the expander's register
    // protocol had not been transcribed; the C6 SDIO hotfix had to work it out
    // for WLAN_PWR_EN, so it is implemented now (see hal/io_expander.h).
    //
    // The INT line doubles as the GT911's I2C address strap, sampled while
    // reset is released: INT high selects 0x14, INT low selects 0x5D. This
    // driver addresses the panel at TAB5_TOUCH_I2C_ADDR (0x14), so INT must be
    // driven HIGH across the release edge, then handed back as an input. Get
    // this wrong and the panel answers at the other address and looks dead.
    // Timing (assert / 10ms / release / 50ms) follows the BSP.
    pinMode(TAB5_TOUCH_INT_GPIO, OUTPUT);
    digitalWrite(TAB5_TOUCH_INT_GPIO, HIGH);
    bool reset_ok = tab5_ioexp::set_output(TAB5_TOUCH_RST_IOEXP_I2C_ADDR,
                                           TAB5_TOUCH_RST_IOEXP_BIT, false);
    delay(10);
    reset_ok = tab5_ioexp::set_output(TAB5_TOUCH_RST_IOEXP_I2C_ADDR,
                                      TAB5_TOUCH_RST_IOEXP_BIT, true) && reset_ok;
    delay(50);
    pinMode(TAB5_TOUCH_INT_GPIO, INPUT);
    if (!reset_ok) {
        Serial.println("quarky-tab5: touch reset via 0x43 IO-expander FAILED");
    }

    // Probe up front so availability is known before loop() starts, rather
    // than being discovered by an error every 5ms forever.
    s_consecutive_failures = 0;
    if (i2cProbe(TAB5_TOUCH_I2C_ADDR)) {
        s_addr = TAB5_TOUCH_I2C_ADDR;
        s_available = true;
    } else if (i2cProbe(kAltTouchI2CAddr)) {
        s_addr = kAltTouchI2CAddr;
        s_available = true;
    } else {
        s_available = false;
    }

    if (s_available) {
        Serial.printf("quarky-tab5: touch GT911 responding @0x%02X\n", s_addr);
    } else {
        Serial.println("quarky-tab5: touch GT911 NOT RESPONDING at 0x14 or "
                       "0x5D -- touch input DISABLED (display/UI unaffected)");
        // One-shot bus scan, logged only on failure. Cheap, read-only, and it
        // turns "touch is broken" into an actionable fact for whoever picks up
        // touch bring-up: it shows whether the panel is on the bus at all (and
        // proves the bus itself is healthy by listing the IO-expanders).
        Serial.print("quarky-tab5: internal I2C scan:");
        for (uint8_t a = 0x08; a < 0x78; a++) {
            if (i2cProbe(a)) {
                Serial.printf(" 0x%02X", a);
            }
        }
        Serial.println();
    }
}

void TouchGT911::read(int16_t &x, int16_t &y, bool &pressed) {
    // Read GT911's touch status + coordinate registers over I2C per the
    // register map documented in pins_config.h (sourced from the espp BSP's
    // Gt911 driver, itself citing Espressif's esp-bsp esp_lcd_touch_gt911.c).
    pressed = false;
    x = 0;
    y = 0;

    if (!s_available) {
        return; // latched off after repeated I2C failures -- see s_available
    }

    uint8_t status = 0;
    if (!i2cReadRegister(s_addr, TAB5_TOUCH_REG_STATUS, &status, 1)) {
        // I2C error (touch not present / not reset / bus wedged). Tolerate a
        // few transients, then give up permanently rather than re-erroring
        // every 5ms for the rest of the boot.
        if (++s_consecutive_failures >= kMaxConsecutiveFailures) {
            s_available = false;
            Serial.println("quarky-tab5: touch GT911 stopped responding -- "
                           "touch input DISABLED for this boot "
                           "(display/UI unaffected)");
        }
        return;
    }
    s_consecutive_failures = 0;

    if ((status & 0x80) == 0) {
        // Bit 7 clear: no new data ready. Nothing to clear/sync either.
        return;
    }

    if ((status & 0x10) == 0x10) {
        // Only the home key was pressed; no coordinate data this cycle.
        // (Home button state is not exposed by the ITouch interface.)
        i2cWriteRegisterU8(s_addr, TAB5_TOUCH_REG_STATUS, 0x00);
        return;
    }

    uint8_t num_points = status & 0x0F;
    if (num_points == 0) {
        i2cWriteRegisterU8(s_addr, TAB5_TOUCH_REG_STATUS, 0x00);
        return;
    }

    // First contact only -- ITouch is a single-point interface (LVGL's
    // pointer indev only needs one point). Layout per contact record:
    // {uint8_t track_id; uint16_t x; uint16_t y; uint16_t area; uint8_t
    // reserved;} = 8 bytes, x/y little-endian on the wire.
    uint8_t point[TAB5_TOUCH_CONTACT_SIZE] = {0};
    if (!i2cReadRegister(s_addr, TAB5_TOUCH_REG_POINT_1, point,
                          sizeof(point))) {
        return;
    }

    uint16_t raw_x = static_cast<uint16_t>(point[1]) | (static_cast<uint16_t>(point[2]) << 8);
    uint16_t raw_y = static_cast<uint16_t>(point[3]) | (static_cast<uint16_t>(point[4]) << 8);
    x = static_cast<int16_t>(raw_x);
    y = static_cast<int16_t>(raw_y);
    pressed = true;

    // Sync/clear signal so the GT911 knows the host consumed this report.
    i2cWriteRegisterU8(s_addr, TAB5_TOUCH_REG_STATUS, 0x00);
}
