#include "battery.h"
#include "../../boards/tab5/pins_config.h"
#include <Wire.h>
#include <Arduino.h>

// See battery.h for the full citation block (I2C address, chip-ID register,
// config register derivation, bus-voltage scaling, percentage formula --
// every one of these fetched directly from M5Stack's own official
// M5Unified library, 2026-08-20).

namespace {

constexpr uint8_t kIna226I2cAddr = 0x41; // Power_Class.hpp, ESP32P4/Tab5 branch
constexpr uint8_t kRegConfig = 0x00;      // INA226_CONFIG
constexpr uint8_t kRegBusVoltage = 0x02;  // INA226_BUS_V
constexpr uint8_t kRegChipId = 0xFF;      // INA226_Class.cpp begin()
constexpr uint16_t kExpectedChipId = 0x2260;

// Real Tab5 config fields (Power_Class.cpp, board_M5Tab5 case) reproduced as
// their raw 3-bit codes (INA226_Class.hpp enum values) since this driver has
// no dependency on M5Unified's C++ types -- only the real numeric values:
//   Sampling::Rate16 = 0b010, ConversionTime::US_1100 = 0b100,
//   Mode::ShuntAndBus = 0b111 (continuous shunt+bus conversion).
constexpr uint16_t kSamplingRate16       = 0b010;
constexpr uint16_t kConversionTime1100us = 0b100; // used for BOTH bus and shunt on Tab5
constexpr uint16_t kModeShuntAndBus      = 0b111;

// Packed exactly as INA226_Class.cpp's config():
//   value = sampling_rate<<9 | bus_conversion_time<<6 | shunt_conversion_time<<3 | mode
constexpr uint16_t kIna226ConfigValue =
    (kSamplingRate16 << 9) | (kConversionTime1100us << 6) |
    (kConversionTime1100us << 3) | kModeShuntAndBus;
static_assert(kIna226ConfigValue == 0x0527,
              "Tab5 INA226 config register arithmetic changed -- recheck "
              "the citation in battery.h before changing this constant");

// Percentage formula constants -- Power_Class.cpp getBatteryLevel(),
// board_M5Tab5 case: `(mv - 3300) * 100 / (float)(4150 - 3350)`.
constexpr float kPercentFloorMv = 3300.0f;
constexpr float kPercentRangeDenomMv = (float)(4150 - 3350); // == 800, NOT 850
// Binds the actual constant used by percent() below, not a private literal --
// a static_assert((4150-3350)==800) here would be a tautology over its own
// operands and would NOT catch a future edit to kPercentRangeDenomMv itself
// (e.g. changing its initializer to (4150-3300) silently passes such an
// assert). Review caught this distinction; fixed to reference the real
// symbol so the regression the comment above warns about actually trips it.
static_assert(kPercentFloorMv == 3300.0f && kPercentRangeDenomMv == 800.0f,
              "recheck this task's percentage-formula citation before "
              "changing these constants -- a secondary summary of the real "
              "M5Unified source paraphrased the range denominator as 850 "
              "during this task's own research and that was wrong; verified "
              "against the raw fetched source directly");

// ---------------------------------------------------------------------------
// I2C helpers. INA226's register address is ONE byte (unlike GT911/ST7123's
// 16-bit register pointer in touch_tab5.cpp's i2cReadRegister()), followed by
// a 16-bit BIG-ENDIAN data word -- INA226_Class.cpp readRegister16()/
// writeRegister16(): `buf[0] << 8 | buf[1]` on read, `{data>>8, data&0xFF}`
// on write.
// ---------------------------------------------------------------------------

bool i2cReadRegister16(uint8_t addr, uint8_t reg, uint16_t *val_out) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) { // repeated start, keep the bus held
        return false;
    }
    size_t got = Wire.requestFrom(static_cast<int>(addr), static_cast<int>(2));
    if (got != 2) {
        return false;
    }
    uint8_t hi = Wire.read();
    uint8_t lo = Wire.read();
    *val_out = (static_cast<uint16_t>(hi) << 8) | lo;
    return true;
}

bool i2cWriteRegister16(uint8_t addr, uint8_t reg, uint16_t value) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    Wire.write(static_cast<uint8_t>(value >> 8));
    Wire.write(static_cast<uint8_t>(value & 0xFF));
    return Wire.endTransmission() == 0;
}

} // namespace

bool Battery::init() {
    // Internal bus -- see battery.h header comment for why this chip lives
    // here rather than on the external HY2.0 PORT.A bus. Wire.begin() is
    // idempotent (display/touch/io_expander already call it); matches every
    // other internal-bus HAL module's own bring-up (touch_tab5.cpp, etc.).
    Wire.begin(TAB5_INTERNAL_I2C_SDA_GPIO, TAB5_INTERNAL_I2C_SCL_GPIO);

    uint16_t id = 0;
    if (!i2cReadRegister16(kIna226I2cAddr, kRegChipId, &id)) {
        Serial.printf("quarky-tab5: battery HAL: INA226 @0x%02X did not "
                      "answer chip-ID register 0x%02X at all -- not "
                      "detected on the internal I2C bus\n",
                      kIna226I2cAddr, kRegChipId);
        init_ok_ = false;
        return false;
    }
    if (id != kExpectedChipId) {
        Serial.printf("quarky-tab5: battery HAL: *** something answered at "
                      "0x%02X but chip-ID register 0x%02X = 0x%04X, expected "
                      "0x%04X -- this is NOT an INA226, do not trust it ***\n",
                      kIna226I2cAddr, kRegChipId, id, kExpectedChipId);
        init_ok_ = false;
        return false;
    }

    // Chip identity confirmed -- now write the real Tab5 config so the chip
    // runs continuous shunt+bus conversion (see kIna226ConfigValue's
    // derivation above) rather than relying on an unverified power-on-reset
    // default.
    bool cfg_ok = i2cWriteRegister16(kIna226I2cAddr, kRegConfig, kIna226ConfigValue);
    Serial.printf("quarky-tab5: battery HAL: INA226 @0x%02X chip-ID confirmed "
                  "(0x%04X); config register (0x%02X) write 0x%04X: %s\n",
                  kIna226I2cAddr, id, kRegConfig, kIna226ConfigValue,
                  cfg_ok ? "OK" : "FAILED");
    init_ok_ = cfg_ok;

    // Log the raw bus-voltage reading once at bring-up, ahead of the deferred
    // hardware checkpoint: an unclamped mV number distinguishes "no pack,
    // reading USB-only VBUS" from "a real 2S pack" immediately, where the
    // clamped percent() alone cannot (see battery.h's disclosed presence-
    // detection gap). Best-effort -- init_ok_ is already latched above either
    // way, this is diagnostic only.
    if (init_ok_) {
        int32_t mv = 0;
        if (bus_voltage_mv(&mv)) {
            // Total 2S pack voltage, not per-cell (percent() applies the /2
            // per-cell step separately -- see its own comment). A real pack
            // reads roughly 6600-8300 mV across its usable range (2 * the
            // 3300-4150 mV/cell floor/ceiling the percentage formula uses);
            // this raw number is logged specifically so the deferred
            // hardware checkpoint can distinguish a real pack from USB-only
            // VBUS, which the clamped percentage alone cannot (battery.h's
            // disclosed presence-detection gap).
            Serial.printf("quarky-tab5: battery HAL: raw bus voltage %ld mV "
                          "total pack (real 2S pack: ~6600-8300 mV range)\n",
                          (long)mv);
        }
    }
    return init_ok_;
}

bool Battery::bus_voltage_mv(int32_t *mv_out) {
    if (!init_ok_) return false;
    uint16_t raw = 0;
    if (!i2cReadRegister16(kIna226I2cAddr, kRegBusVoltage, &raw)) {
        return false;
    }
    // INA226_Class.cpp getBusVoltage(): `(int16_t)readRegister16(...) *
    // 0.00125f` -- volts, 1.25 mV/LSB, signed raw value. Converted to mV here.
    int16_t signed_raw = static_cast<int16_t>(raw);
    *mv_out = static_cast<int32_t>(static_cast<float>(signed_raw) * 1.25f);
    return true;
}

bool Battery::percent(int *percent_out) {
    if (!init_ok_) return false;
    uint16_t raw = 0;
    if (!i2cReadRegister16(kIna226I2cAddr, kRegBusVoltage, &raw)) {
        return false;
    }
    int16_t signed_raw = static_cast<int16_t>(raw);

    // Ported directly from Power_Class.cpp getBatteryLevel(), board_M5Tab5
    // case -- see battery.h's citation block for the exact source lines.
    // Kept as the same float expressions as the real source (not
    // re-derived into an integer-only formula) so this stays a transcription,
    // not a re-algebra-ed reimplementation that could introduce its own bug.
    float bus_volts = static_cast<float>(signed_raw) * 0.00125f; // getBusVoltage()
    float mv = bus_volts * 500.0f; // real source's own comment: "2S Li-Po ( * 1000 / 2 == * 500)"
    int level = static_cast<int>((mv - kPercentFloorMv) * 100.0f / kPercentRangeDenomMv);

    if (level < 0) level = 0;
    else if (level >= 100) level = 100;

    *percent_out = level;
    return true;
}
