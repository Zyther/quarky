#include "touch_tab5.h"
#include "io_expander.h"
#include "../../boards/tab5/pins_config.h"
#include <Arduino.h>
#include <Wire.h>

// ===========================================================================
// Why this file exists in this shape
// ===========================================================================
//
// Task 6 shipped a GT911-only touch driver. On this board it found nothing at
// 0x14 or 0x5D and disabled itself -- correctly, because THIS UNIT HAS NO
// GT911. It is the same class of bug the display hotfix hit: the community
// espp BSP models only one of the two Tab5 hardware revisions, and this unit
// is the other one.
//
// The display hotfix established (by reading the touch firmware-version
// register at I2C 0x55, per M5Stack's own M5GFX) that this panel is an
// **ST7121**. The ST7121/ST7123 are TDDI parts -- Touch and Display Driver
// Integration -- meaning the capacitive touch controller is not a separate
// chip at all. It is inside the same silicon as the MIPI-DSI display driver
// and is read over I2C at the panel's own address, 0x55, via a vendor
// register range. There was never going to be anything at 0x14/0x5D.
//
// SOURCES (all fetched 2026-08-08, all agreeing with each other):
//
// 1. m5stack/M5GFX -- the vendor's own library for this exact board.
//    `src/M5GFX.cpp`, `board_M5Tab5` bring-up (L2778-2823): the touch object
//    chosen is a function of the detected panel, and only the ILI9881C
//    revision gets a GT911:
//
//        if (hit_ili9881)      { _touch_last.reset(new Touch_GT911());   ... }
//        else if (hit_st7121)  { _touch_last.reset(new Touch_ST7123());  ... }
//        else if (hit_st7123)  { _touch_last.reset(new Touch_ST7123());  ... }
//
//    i.e. an ST7121 uses `Touch_ST7123`, whose
//    `src/lgfx/v1/platforms/esp32p4/Touch_ST7123.hpp` declares
//    `static constexpr const uint8_t default_addr = 0x55;` -- the *same*
//    address as the display controller. One chip, two register ranges.
//
// 2. m5stack/M5GFX `src/lgfx/v1/platforms/esp32p4/Touch_ST7123.cpp` --
//    the register map and read sequence implemented below, verbatim:
//        ST7123_FW_VERSION_REG     = 0x0000
//        ST7123_FW_REVISION_REG    = 0x000C
//        ST7123_MAX_X_COORD_H_REG  = 0x0005   (then _L 0x0006, Y_H 0x0007,
//                                              Y_L 0x0008)
//        ST7123_MAX_TOUCHES_REG    = 0x0009
//        ST7123_REPORT_COORD_0_REG = 0x0014
//    plus `getTouchRaw()`: read 1 byte from 0x0010, test `with_coord`, then
//    read max_touches * sizeof(touch_report_t) bytes from 0x0014.
//
// 3. esp-cpp/espp `components/st7123touch/include/st7123touch.hpp` -- an
//    INDEPENDENT implementation of the same protocol, used here purely as a
//    cross-check. Its header comment states the sequence in prose and its
//    code agrees with M5GFX byte-for-byte on every register number, on the
//    7-byte report stride, on `ADV_INFO_WITH_COORD = (1 << 3)`, and on the
//    field packing `x = ((p[0] & 0x3F) << 8) | p[1]; y = (p[2] << 8) | p[3]`.
//    Two independent sources agreeing is why no register below is a guess.
//
// 4. M5Stack's own product documentation for the Tab5
//    (docs.m5stack.com/en/core/Tab5) lists the internal I2C bus as
//    0x10 ES8388, 0x32 RX8130CE, 0x40 ES7210, 0x41 INA226, 0x43/0x44
//    PI4IOE5V6408, 0x55 "ST7123 or ST7121 (display/touch driver)", 0x68
//    BMI270 -- and notes that units before 2025-10-14 used a GT911 at 0x14
//    instead. That is the two-revision split, stated by the vendor.
//
// ---------------------------------------------------------------------------
// The second bug: the old driver's TP_RST pulse was actively harmful here
// ---------------------------------------------------------------------------
//
// espp's st7123touch.hpp carries this warning in its class documentation:
//
//   "The ST7123's touch engine is gated by the LCD reset (LCD_RST) line, NOT
//    the TP_RST line used by standalone touch controllers such as the GT911.
//    When used in a system that has a separate TP_RST signal (e.g. M5Stack
//    Tab5), do NOT toggle TP_RST for this chip - doing so may knock the touch
//    I2C endpoint offline."
//
// The old driver pulsed TP_RST (IO-expander 0x43 P5) unconditionally, *after*
// the display had already been initialised. So even once the right address
// was known, probing 0x55 after that pulse could have failed. Hence the
// ordering below: the ST TDDI endpoint is probed FIRST and the TP_RST /
// INT-strap dance is confined to the GT911 fallback path, which is the only
// path that needs it.
//
// M5GFX corroborates that no touch reset is needed for the ST part: it sets
// `cfg.pin_rst = -1` for the touch object on Tab5 (M5GFX.cpp L2850), so
// `Touch_ST7123::init()`'s reset block is skipped entirely. The chip is
// already out of reset because DisplayTab5::init() pulsed LCD_RST.
// ===========================================================================

namespace {

// ---------------------------------------------------------------------------
// I2C helpers. Both touch protocols use a 16-bit, big-endian register pointer
// followed by a repeated-START read, so one pair of helpers serves both.
// ---------------------------------------------------------------------------

bool i2cReadRegister(uint8_t addr, uint16_t reg, uint8_t *buf, size_t len) {
    Wire.beginTransmission(addr);
    Wire.write(static_cast<uint8_t>(reg >> 8));
    Wire.write(static_cast<uint8_t>(reg & 0xFF));
    if (Wire.endTransmission(false) != 0) { // repeated start, keep the bus held
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

// Read-only presence check: address the device and see whether anything ACKs.
bool i2cProbe(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

// ---------------------------------------------------------------------------
// Latched availability.
//
// PRESERVED FROM THE PREVIOUS DRIVER, DELIBERATELY. Once touch has proven
// unreachable we stop talking to it entirely. Without this, a panel that does
// not answer produces two framework error lines per read() -- roughly 30
// lines/second -- which floods the serial console, drowns every other
// diagnostic, and burns bus time on a device that is not there. That was
// observed on real hardware during the C6 hotfix. Degrade quietly: log once,
// then no-op. Every failure path below funnels into this.
// ---------------------------------------------------------------------------
TouchTab5::Backend s_backend = TouchTab5::Backend::None;
uint8_t s_addr = 0;
uint8_t s_consecutive_failures = 0;
constexpr uint8_t kMaxConsecutiveFailures = 10;

// GT911's alternate address (INT-pin strap: high -> 0x14, low -> 0x5D).
constexpr uint8_t kGt911AltI2CAddr = 0x5D;

// ---------------------------------------------------------------------------
// ST7121/ST7123 TDDI touch
// ---------------------------------------------------------------------------

// Vendor's `_read_fw_info()` (M5GFX Touch_ST7123.cpp): read the firmware
// version, the 4-byte revision, and the max-coordinate/max-touch block, and
// treat an all-zero response as "not really there". A device that merely ACKs
// its address is not proof of a working touch engine; a plausible, non-zero
// firmware descriptor is. This is the identification step, and it is also what
// makes the log line below a real measurement rather than an assumption.
struct StFwInfo {
    uint8_t version;
    uint8_t revision[4];
    uint16_t max_x;
    uint16_t max_y;
    uint8_t max_touches;
};

bool stReadFwInfo(StFwInfo &out) {
    uint8_t version = 0;
    uint8_t revision[4] = {0};
    uint8_t block[5] = {0}; // max_x_h, max_x_l, max_y_h, max_y_l, max_touches

    if (!i2cReadRegister(TAB5_TOUCH_ST_I2C_ADDR, TAB5_TOUCH_ST_REG_FW_VERSION, &version, 1) ||
        !i2cReadRegister(TAB5_TOUCH_ST_I2C_ADDR, TAB5_TOUCH_ST_REG_FW_REVISION, revision, 4) ||
        !i2cReadRegister(TAB5_TOUCH_ST_I2C_ADDR, TAB5_TOUCH_ST_REG_MAX_X_H, block, sizeof(block))) {
        return false;
    }

    uint32_t sum = version;
    for (uint8_t b : revision) sum += b;
    for (uint8_t b : block) sum += b;
    if (sum == 0) {
        // Everything read back as zero. The vendor treats this as "not
        // present" -- an ACK with no firmware behind it.
        return false;
    }

    out.version = version;
    for (int i = 0; i < 4; i++) out.revision[i] = revision[i];
    out.max_x = static_cast<uint16_t>(block[0]) << 8 | block[1];
    out.max_y = static_cast<uint16_t>(block[2]) << 8 | block[3];
    out.max_touches = block[4];
    return true;
}

// ---------------------------------------------------------------------------
// Native panel coordinates -> logical UI coordinates.
//
// The touch engine reports in the panel's NATIVE portrait space. M5GFX
// configures the Tab5 touch object with `cfg.x_max = 719; cfg.y_max = 1279;`
// and `cfg.offset_rotation = 0` against a panel whose own `offset_rotation`
// is also 0 (M5GFX.cpp L2840, L2855-2861) -- i.e. touch axes and panel axes
// are already aligned, with no flip or swap of their own.
//
// DisplayTab5 presents a rotated LOGICAL surface to LVGL and rotates pixels
// on the way down. Touch therefore has to run that same transform backwards.
// display_tab5.cpp documents the forward mapping as, for logical (lx, ly):
//
//     90  (CCW): native x = ly,            native y = LW-1-lx
//     270 (CW):  native x = LH-1-ly,       native y = lx
//
// Inverting those gives exactly what is implemented below. Getting this wrong
// does not look like "touch is broken" -- it looks like taps landing in the
// wrong place, which is much harder to diagnose, so the derivation is spelled
// out rather than left implicit.
// ---------------------------------------------------------------------------
inline int16_t clampToRange(int32_t v, int32_t lo, int32_t hi) {
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    return static_cast<int16_t>(v);
}

void mapNativeToLogical(uint16_t nx, uint16_t ny, int16_t &lx, int16_t &ly) {
    // Clamp into the native panel first: the x field is only 6+8 bits wide, so
    // a corrupted report can carry a nonsense magnitude.
    const int32_t cnx = clampToRange(nx, 0, TAB5_PANEL_NATIVE_WIDTH - 1);
    const int32_t cny = clampToRange(ny, 0, TAB5_PANEL_NATIVE_HEIGHT - 1);

#if TAB5_DISPLAY_ROTATION == 90
    lx = clampToRange((TAB5_DISP_WIDTH - 1) - cny, 0, TAB5_DISP_WIDTH - 1);
    ly = clampToRange(cnx, 0, TAB5_DISP_HEIGHT - 1);
#elif TAB5_DISPLAY_ROTATION == 270
    lx = clampToRange(cny, 0, TAB5_DISP_WIDTH - 1);
    ly = clampToRange((TAB5_DISP_HEIGHT - 1) - cnx, 0, TAB5_DISP_HEIGHT - 1);
#else // TAB5_DISPLAY_ROTATION == 0: logical space *is* native space
    lx = clampToRange(cnx, 0, TAB5_DISP_WIDTH - 1);
    ly = clampToRange(cny, 0, TAB5_DISP_HEIGHT - 1);
#endif
}

// Emit a labelled scan of the internal bus. Only called when touch could not
// be brought up (or when TAB5_TOUCH_I2C_CENSUS is set), so a healthy boot
// stays quiet. Labels are from M5Stack's own Tab5 documentation; anything not
// in that table is reported honestly as unknown rather than guessed at.
const char *labelForI2CAddr(uint8_t a) {
    switch (a) {
        case 0x10: return "ES8388 audio codec";
        case 0x14: return "GT911 touch (ILI9881 revision)";
        case 0x32: return "RX8130CE RTC";
        case 0x40: return "ES7210 mic ADC";
        case 0x41: return "INA226 power monitor";
        case 0x43: return "PI4IOE5V6408 IO-exp #1 (LCD_RST/TP_RST/SPK_EN)";
        case 0x44: return "PI4IOE5V6408 IO-exp #2 (WLAN_PWR_EN/charger)";
        case 0x55: return "ST7121/ST7123 panel + integrated touch";
        case 0x5D: return "GT911 touch, alt strap";
        case 0x68: return "BMI270 IMU";
        default:   return "unidentified";
    }
}

void scanInternalBus() {
    Serial.println("quarky-tab5: internal I2C census:");
    for (uint8_t a = 0x08; a < 0x78; a++) {
        if (i2cProbe(a)) {
            Serial.printf("quarky-tab5:   0x%02X  %s\n", a, labelForI2CAddr(a));
        }
    }
}

} // namespace

// ===========================================================================
// init()
// ===========================================================================

void TouchTab5::init() {
    // SDA/SCL are the BSP's internal I2C bus pins, not the Arduino variant
    // defaults -- see TAB5_INTERNAL_I2C_SDA_GPIO/SCL_GPIO. Same bus the
    // display's reset IO-expander and the panel itself live on. Wire.begin()
    // is idempotent; DisplayTab5::init() has already called it.
    Wire.begin(TAB5_INTERNAL_I2C_SDA_GPIO, TAB5_INTERNAL_I2C_SCL_GPIO);

    // 400 kHz, which is what M5GFX uses for the Tab5 touch object
    // (`cfg.freq = 400000`, M5GFX.cpp L2854). The previous driver left this at
    // Arduino's 100 kHz default and documented that as a conservative choice;
    // it is not conservative any more, because the ST read path can pull
    // max_touches * 7 = up to 70 bytes in a single transaction while a finger
    // is down. At 100 kHz that is ~6.5 ms of blocking bus time inside the LVGL
    // input callback; at the vendor's 400 kHz it is ~1.6 ms. Every device on
    // this bus is rated for at least 400 kHz (the BSP itself runs it at 1 MHz),
    // and this is set after DisplayTab5::init() has finished all of its own
    // I2C, so panel bring-up is unaffected.
    Wire.setClock(TAB5_TOUCH_I2C_FREQ_HZ);

    s_backend = Backend::None;
    s_addr = 0;
    s_consecutive_failures = 0;

    // --- 1. ST7121/ST7123 integrated (TDDI) touch, at the panel's address ---
    //
    // Probed FIRST, and probed WITHOUT touching TP_RST. Two reasons:
    //   * this is the revision this unit actually is, so it is the fast path;
    //   * espp's st7123touch driver warns that toggling TP_RST on a TDDI part
    //     can knock its touch I2C endpoint offline (see the header comment).
    //     Doing the GT911 reset dance first would risk breaking the very thing
    //     we are trying to find.
    // The chip is already out of reset: DisplayTab5::init() pulsed LCD_RST,
    // which is what gates the ST touch engine.
    //
    // Retry count follows the vendor: `Touch_ST7123::init()` retries
    // `_read_fw_info()` up to 6 times before giving up.
    StFwInfo fw = {};
    bool st_ok = false;
    for (int attempt = 0; attempt < 6 && !st_ok; attempt++) {
        st_ok = stReadFwInfo(fw);
        if (!st_ok && attempt < 5) {
            delay(10);
        }
    }

    if (st_ok) {
        s_backend = Backend::StTddi;
        s_addr = TAB5_TOUCH_ST_I2C_ADDR;
        Serial.printf("quarky-tab5: touch ST TDDI @0x%02X OK -- fw %u (%u.%u.%u.%u), "
                      "max %ux%u, %u points\n",
                      s_addr, fw.version, fw.revision[0], fw.revision[1], fw.revision[2],
                      fw.revision[3], fw.max_x, fw.max_y, fw.max_touches);
#if TAB5_TOUCH_I2C_CENSUS
        scanInternalBus();
#endif
        return;
    }

    // --- 2. Standalone GT911 (ILI9881-revision boards) -----------------------
    //
    // Only reached when there is no TDDI touch engine, i.e. on the other
    // hardware revision, where TP_RST and the INT address strap are real and
    // required. Preserved from the previous driver unchanged.
    //
    // The INT line doubles as the GT911's I2C address strap, sampled while
    // reset is released: INT high selects 0x14, INT low selects 0x5D. Drive it
    // HIGH across the release edge, then hand it back as an input. Timing
    // (assert / 10 ms / release / 50 ms) follows the BSP.
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

    if (i2cProbe(TAB5_TOUCH_I2C_ADDR)) {
        s_backend = Backend::Gt911;
        s_addr = TAB5_TOUCH_I2C_ADDR;
    } else if (i2cProbe(kGt911AltI2CAddr)) {
        s_backend = Backend::Gt911;
        s_addr = kGt911AltI2CAddr;
    }

    if (s_backend == Backend::Gt911) {
        Serial.printf("quarky-tab5: touch GT911 responding @0x%02X\n", s_addr);
#if TAB5_TOUCH_I2C_CENSUS
        scanInternalBus();
#endif
        return;
    }

    // --- 3. Nothing found: degrade gracefully -------------------------------
    Serial.println("quarky-tab5: touch NOT FOUND -- no ST TDDI engine at 0x55 and no "
                   "GT911 at 0x14/0x5D -- touch input DISABLED (display/UI unaffected)");
    // One-shot labelled scan, logged only on failure. Cheap, read-only, and it
    // turns "touch is broken" into an actionable fact: it shows whether the
    // panel is on the bus at all, and proves the bus itself is healthy by
    // listing the IO-expanders.
    scanInternalBus();
}

TouchTab5::Backend TouchTab5::backend() const { return s_backend; }
bool TouchTab5::available() const { return s_backend != Backend::None; }

// ===========================================================================
// read()
// ===========================================================================

namespace {

// Funnel for every I2C failure in read(): tolerate a few transients, then give
// up permanently rather than re-erroring forever. See s_backend comment.
void noteFailure() {
    if (++s_consecutive_failures >= kMaxConsecutiveFailures) {
        s_backend = TouchTab5::Backend::None;
        Serial.println("quarky-tab5: touch stopped responding -- touch input "
                       "DISABLED for this boot (display/UI unaffected)");
    }
}

// --- ST7121/ST7123 TDDI read path ------------------------------------------
//
// Transcribed from M5GFX `Touch_ST7123::getTouchRaw()` and cross-checked
// against espp `St7123Touch::update()`; both are quoted at the top of this
// file. Sequence:
//   1. read 1 byte from ADV_INFO (0x0010); bit 3 = with_coord
//   2. if set: read max_touches (0x0009), then max_touches * 7 bytes from
//      REPORT_COORD_0 (0x0014)
//   3. per 7-byte report: valid = byte[0] & 0x80,
//      x = ((byte[0] & 0x3F) << 8) | byte[1], y = (byte[2] << 8) | byte[3]
//
// M5GFX expresses the report as a bitfield struct; the byte/mask form here is
// the same layout written out explicitly (GCC packs bitfields from the LSB on
// this little-endian target, so `x_h:6, reserved_6:1, valid:1` is bits 0-5,
// 6, 7 of byte 0). espp's independent implementation uses exactly these masks,
// which is the cross-check that the bitfield reading is right.
bool readStTddi(int16_t &x, int16_t &y, bool &pressed) {
    uint8_t adv_info = 0;
    if (!i2cReadRegister(TAB5_TOUCH_ST_I2C_ADDR, TAB5_TOUCH_ST_REG_ADV_INFO, &adv_info, 1)) {
        return false;
    }

    if ((adv_info & TAB5_TOUCH_ST_ADV_WITH_COORD) == 0) {
        // No coordinate data this cycle -> finger is up. Not an error.
        return true;
    }

    uint8_t max_touches = 0;
    if (!i2cReadRegister(TAB5_TOUCH_ST_I2C_ADDR, TAB5_TOUCH_ST_REG_MAX_TOUCHES, &max_touches, 1)) {
        return false;
    }
    if (max_touches == 0) {
        return true; // firmware reports no slots; treat as finger up
    }
    if (max_touches > TAB5_TOUCH_ST_MAX_POINTS) {
        max_touches = TAB5_TOUCH_ST_MAX_POINTS;
    }

    uint8_t data[TAB5_TOUCH_ST_MAX_POINTS * TAB5_TOUCH_ST_REPORT_SIZE] = {0};
    const size_t want = static_cast<size_t>(max_touches) * TAB5_TOUCH_ST_REPORT_SIZE;
    if (!i2cReadRegister(TAB5_TOUCH_ST_I2C_ADDR, TAB5_TOUCH_ST_REG_REPORT_0, data, want)) {
        return false;
    }

    // First valid contact only -- ITouch is a single-point interface (LVGL's
    // pointer indev only needs one). Slots can be sparse, so scan rather than
    // assuming slot 0 is the live one.
    for (uint8_t i = 0; i < max_touches; i++) {
        const uint8_t *p = &data[i * TAB5_TOUCH_ST_REPORT_SIZE];
        if ((p[0] & 0x80) == 0) {
            continue; // slot not valid
        }
        const uint16_t nx = static_cast<uint16_t>((p[0] & 0x3F) << 8) | p[1];
        const uint16_t ny = static_cast<uint16_t>(p[2] << 8) | p[3];
        mapNativeToLogical(nx, ny, x, y);
        pressed = true;
        break;
    }
    return true;
}

// --- GT911 read path --------------------------------------------------------
//
// Unchanged from the previous driver (espp's Gt911, itself citing Espressif's
// esp-bsp esp_lcd_touch_gt911.c). Retained for ILI9881-revision boards; it is
// dead code on this unit and cannot be exercised here.
bool readGt911(int16_t &x, int16_t &y, bool &pressed) {
    uint8_t status = 0;
    if (!i2cReadRegister(s_addr, TAB5_TOUCH_REG_STATUS, &status, 1)) {
        return false;
    }

    if ((status & 0x80) == 0) {
        return true; // bit 7 clear: no new data ready, nothing to clear either
    }

    if ((status & 0x10) == 0x10) {
        // Only the home key was pressed; no coordinate data this cycle.
        // (Home button state is not exposed by the ITouch interface.)
        i2cWriteRegisterU8(s_addr, TAB5_TOUCH_REG_STATUS, 0x00);
        return true;
    }

    const uint8_t num_points = status & 0x0F;
    if (num_points == 0) {
        i2cWriteRegisterU8(s_addr, TAB5_TOUCH_REG_STATUS, 0x00);
        return true;
    }

    // Contact record: {uint8_t track_id; uint16_t x; uint16_t y; uint16_t
    // area; uint8_t reserved;} = 8 bytes, x/y little-endian on the wire.
    uint8_t point[TAB5_TOUCH_CONTACT_SIZE] = {0};
    if (!i2cReadRegister(s_addr, TAB5_TOUCH_REG_POINT_1, point, sizeof(point))) {
        return false;
    }

    const uint16_t nx = static_cast<uint16_t>(point[1]) | (static_cast<uint16_t>(point[2]) << 8);
    const uint16_t ny = static_cast<uint16_t>(point[3]) | (static_cast<uint16_t>(point[4]) << 8);
    mapNativeToLogical(nx, ny, x, y);
    pressed = true;

    // Sync/clear so the GT911 knows the host consumed this report.
    i2cWriteRegisterU8(s_addr, TAB5_TOUCH_REG_STATUS, 0x00);
    return true;
}

} // namespace

void TouchTab5::read(int16_t &x, int16_t &y, bool &pressed) {
    pressed = false;
    x = 0;
    y = 0;

    bool ok = true;
    switch (s_backend) {
        case Backend::StTddi: ok = readStTddi(x, y, pressed); break;
        case Backend::Gt911:  ok = readGt911(x, y, pressed);  break;
        case Backend::None:
        default:
            return; // latched off -- see the s_backend comment
    }

    if (!ok) {
        noteFailure();
        pressed = false;
        x = 0;
        y = 0;
        return;
    }
    s_consecutive_failures = 0;

#if TAB5_TOUCH_TRACE
    // Opt-in trace. Prints only on a press/release edge or a moved point, so a
    // finger held still does not flood the console -- the failure mode this
    // driver's whole latching design exists to avoid.
    static bool s_last_pressed = false;
    static int16_t s_last_x = -1, s_last_y = -1;
    if (pressed != s_last_pressed || (pressed && (x != s_last_x || y != s_last_y))) {
        Serial.printf("quarky-tab5: touch %s (%d,%d)\n", pressed ? "DOWN" : "up  ", x, y);
        s_last_pressed = pressed;
        s_last_x = x;
        s_last_y = y;
    }
#endif
}
