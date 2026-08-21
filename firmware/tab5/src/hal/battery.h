#pragma once
#include <cstdint>

// Real HAL for the Tab5's battery/power monitoring. Phase 3 Task 23.
//
// Before this task, the status bar's battery percentage
// (ui/shell.cpp's build(), pre-fix) was a PERMANENTLY HARDCODED
// "Battery: --%" string -- a plain local lv_label_t that nothing ever called
// lv_label_set_text() on again. There was no HAL module, no ADC read, no
// fuel-gauge/PMIC access anywhere in this codebase for battery state
// (confirmed by a repo-wide grep for battery/Battery/BAT_ before writing this
// file; the only other hits were unrelated BLE battery-service *spoofing* in
// ble_sourapple.cpp/ble_findmy.cpp/ble_fastpair_exploit.cpp, which fake a
// battery level in an advertised payload and don't read this device's own).
//
// ===========================================================================
// REAL CHIP IDENTITY AND CITATIONS (fetched directly, 2026-08-20, from
// M5Stack's own official M5Unified library -- MIT-licensed, confirmed via the
// file's own header comment "Licensed under the MIT license"):
//   https://github.com/m5stack/M5Unified
//   src/utility/Power_Class.hpp / Power_Class.cpp
//   src/utility/power/INA226_Class.hpp / INA226_Class.cpp
// Every constant below was read directly from those files via
// `gh api repos/m5stack/M5Unified/contents/<path>`, not taken on trust from
// any restatement -- including a second, independent verification pass of
// this same task's own controller-notes research (see this task's report for
// what that re-verification found, including a place where re-deriving the
// percentage formula from the raw source caught a wrong constant in a
// search-engine summary that had been consulted along the way: "4150-3350"
// was paraphrased elsewhere as an 850 mV range; it is arithmetically 800).
//
// THE CHIP: a TI INA226 precision power monitor IC.
//   I2C ADDRESS: 0x41 on Tab5 specifically -- Power_Class.hpp,
//     `#elif defined (CONFIG_IDF_TARGET_ESP32P4)` branch:
//     `INA226_Class Ina226 = { 0x41 };`. NOT the class's own
//     `DEFAULT_ADDRESS = 0x40` (INA226_Class.hpp) -- that default is what a
//     DIFFERENT M5Stack product family (ESP32S3 branch) uses; Tab5 overrides
//     it to 0x41. This also matches this project's own pre-existing
//     documentation: pins_config.h's internal-bus census table and
//     touch_tab5.cpp's labelForI2CAddr() both already independently labelled
//     0x41 "INA226 battery power monitor" (from earlier Phase 1 bring-up
//     research), which is corroborating, not this task's own discovery.
//   BUS: the INTERNAL I2C bus (TAB5_INTERNAL_I2C_SDA_GPIO/SCL_GPIO, GPIO
//     31/32, `Wire`), NOT the external HY2.0 PORT.A bus (`Wire1`) the
//     NFC/RFID2 units use. Inferred from INA226_Class's constructor default
//     (`I2C_Class* i2c = &In_I2C`, the same internal-bus instance the
//     IO-expanders at 0x43/0x44 use in the same board_M5Tab5 case) and
//     confirmed by this project's own pre-existing internal-bus census
//     (pins_config.h, touch_tab5.cpp) already finding 0x41 answering there on
//     real hardware -- not a fresh discovery, but real corroboration that
//     this hypothesis is correct for THIS unit specifically.
//   CHIP-ID REGISTER: 0xFF, expected value 0x2260 -- INA226_Class.cpp
//     `begin()`: `uint16_t id = readRegister16(0xFF); _init = (id == 0x2260);`
//     This is the real, falsifiable acceptance test for this HAL's init(),
//     matching this project's established chip-identification convention
//     (see features/nfc/st25r3916_driver.h's IC-identity-register precedent).
//   REGISTERS USED: INA226_Class.hpp --
//     INA226_CONFIG = 0x00, INA226_BUS_V = 0x02, INA226_CALIBRATION = 0x05.
//   BUS VOLTAGE SCALING: INA226_Class.cpp `getBusVoltage()`:
//     `auto raw = (int16_t)readRegister16(INA226_BUS_V); return raw * 0.00125f;`
//     -- 1.25 mV per LSB, signed 16-bit raw register value.
//   WIRE PROTOCOL: INA226_Class.cpp `readRegister16`/`writeRegister16`: a
//     single 8-bit register address followed by a 16-bit BIG-ENDIAN data word
//     (`buf[0] << 8 | buf[1]` on read; `{data>>8, data&0xFF}` on write) --
//     this is a DIFFERENT addressing width from this project's existing
//     touch_tab5.cpp i2cReadRegister() helper, which uses a 16-bit register
//     pointer (GT911/ST7123 convention); INA226's register address is only
//     one byte, so that helper is not reused here.
//   TAB5 CONFIGURATION REGISTER VALUE: Power_Class.cpp, board_M5Tab5 case
//     inside the power-init switch:
//       cfg.sampling_rate = INA226_Class::Sampling::Rate16;          // 0b010
//       cfg.bus_conversion_time = INA226_Class::ConversionTime::US_1100;   // 0b100
//       cfg.shunt_conversion_time = INA226_Class::ConversionTime::US_1100; // 0b100
//       cfg.mode = INA226_Class::Mode::ShuntAndBus;                 // 0b111
//       cfg.shunt_res = 0.005f;            // 5 mOhm
//       cfg.max_expected_current = 2.0f;   // 2A
//     packed by INA226_Class.cpp `config()`:
//       value = sampling_rate<<9 | bus_conversion_time<<6 | shunt_conversion_time<<3 | mode
//     which for the exact Tab5 values above is a fixed, computable constant
//     -- see kIna226ConfigValue below for the arithmetic. This HAL writes
//     that same real config register value so the chip runs in the same
//     continuous shunt+bus conversion mode M5Unified configures it for,
//     rather than assuming an unverified power-on-reset default.
//   CALIBRATION REGISTER: deliberately NOT written here. It only scales the
//     current/power registers (INA226_CURRENT/INA226_POWER), which this HAL
//     does not read -- bus-voltage-only percentage needs no calibration.
//     (M5Unified's own derivation, for a future task that adds current/power:
//     `current_LSB = max_expected_current / 32768.0f;
//      cal = (uint16_t)(0.00512f / (current_LSB * shunt_res));`.)
//   BATTERY PERCENTAGE FORMULA, 2S Li-Po pack -- Power_Class.cpp
//     `getBatteryLevel()`, board_M5Tab5 case:
//       mv = Ina226.getBusVoltage() * 500;   // comment in the real source:
//                                             // "2S Li-Po ( * 1000 / 2 == * 500)"
//       int level = (mv - 3300) * 100 / (float)(4150 - 3350);
//       // clamped: level<0 -> 0, level>=100 -> 100
//     Verified directly against the raw file content (not a paraphrase):
//     4150 - 3350 IS 800, not 850. A restated/summarized version of this
//     exact source consulted during this task's research independently
//     produced "850" for that same subtraction -- caught only by computing
//     it from the fetched source text rather than trusting the summary.
//     Multiple summarization passes over the same real file made the same
//     arithmetic slip, which is the whole reason this header insists on
//     recomputing it here rather than restating a paraphrase one more time.
//   KNOWN, DISCLOSED UPSTREAM LIMITATION (not silently fixed): M5Unified has
//     explicit battery-presence-detection logic (`Power_Class::_batteryPresent()`)
//     but it is compiled only under
//     `#if defined (CONFIG_IDF_TARGET_ESP32C5) || defined (CONFIG_IDF_TARGET_ESP32C61)`
//     (Power_Class.cpp) -- verified directly from the fetched source, and a
//     more precise citation than this task's own controller notes, which had
//     paraphrased the gated targets as "C6, C5" rather than "C5, C61". There
//     is NO equivalent presence check for ESP32P4/Tab5 in the upstream code
//     this driver ports the percentage formula from. Practical consequence:
//     with no battery pack attached, USB-only VBUS can read a plausible bus
//     voltage on the INA226 (community reports ~4.3V), and percent() below
//     will compute a percentage from it as if a real pack were present --
//     giving a plausible-looking but meaningless number, or clamping to
//     0%/100%. No real, citable presence-detection heuristic for THIS board
//     was found; none is invented here. A future task should not "fix" this
//     silently -- see percent()'s own comment.
// ===========================================================================
class Battery {
public:
    // Brings up the internal I2C bus (idempotent -- Wire.begin() is a no-op
    // if display/touch/etc already called it) and confirms the INA226
    // answers at 0x41 with the real chip-ID value (0xFF == 0x2260), then
    // writes the real Tab5 continuous-conversion config register value.
    // Returns false if the chip never answers or answers with the wrong
    // identity. Callers must not trust bus_voltage_mv()/percent() after a
    // false return -- they will simply keep returning false too, but the
    // caller should still show "--%" rather than call them at all.
    bool init();

    // Whether init() has succeeded. Does not re-probe the bus.
    bool chip_detected() const { return init_ok_; }

    // Real bus-voltage read (INA226_BUS_V, 1.25 mV/LSB), returned as
    // millivolts of the WHOLE 2S pack (raw * 1.25, not yet the per-cell-
    // averaged * 500 scaling percent() below uses -- this is the chip's
    // literal bus voltage node reading). Returns false on any I2C failure;
    // *mv_out is left untouched in that case.
    bool bus_voltage_mv(int32_t *mv_out);

    // Derived battery percentage via M5Unified's own real 2S-pack formula
    // (see header comment above for the exact arithmetic and citations).
    // Clamped to [0, 100]. Returns false (leaving *percent_out untouched) on
    // any I2C failure or if init() has not succeeded -- callers must show
    // "--%" rather than a stale or fabricated number in that case, not
    // silently substitute a guess.
    //
    // See the header's "KNOWN, DISCLOSED UPSTREAM LIMITATION" note: this
    // formula assumes a battery pack is physically present. On USB-only
    // power with no pack attached it can return a plausible-looking but
    // meaningless number rather than an honest "no battery" signal --
    // inherited from M5Unified's own upstream Tab5 code path, which has no
    // presence check for this specific board. Disclosed here rather than
    // silently carried forward without comment.
    bool percent(int *percent_out);

private:
    bool init_ok_ = false;
};
