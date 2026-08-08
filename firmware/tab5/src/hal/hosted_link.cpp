#include "hosted_link.h"

#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>

extern "C" {
#include "esp32-hal-hosted.h"
}

#include "io_expander.h"
#include "../../boards/tab5/pins_config.h"

// Compile-time regression tripwire. BOARD_SDIO_ESP_HOSTED_* comes from the
// selected Arduino variant's pins_arduino.h (pulled in by <Arduino.h>) and is
// what cores/esp32/esp32-hal-hosted.c bakes into its SDIO pin config. If
// platformio.ini is ever pointed back at a stock board -- or the in-repo
// boards/ directory stops being picked up -- these values silently revert to
// the generic ESP32-P4-Function-EV-Board's wrong pins and the board returns to
// crash-looping on every boot, with nothing at build time to say why. Fail the
// build instead. See boards/variants/quarky_tab5_p4/pins_arduino.h.
#ifndef BOARD_HAS_SDIO_ESP_HOSTED
#error "Selected Arduino variant does not define BOARD_HAS_SDIO_ESP_HOSTED -- expected boards/variants/quarky_tab5_p4"
#endif
static_assert(BOARD_SDIO_ESP_HOSTED_CLK == TAB5_C6_SDIO_CLK_GPIO,
              "esp-hosted CLK pin is not the Tab5's -- wrong board variant selected");
static_assert(BOARD_SDIO_ESP_HOSTED_CMD == TAB5_C6_SDIO_CMD_GPIO,
              "esp-hosted CMD pin is not the Tab5's -- wrong board variant selected");
static_assert(BOARD_SDIO_ESP_HOSTED_D0 == TAB5_C6_SDIO_D0_GPIO,
              "esp-hosted D0 pin is not the Tab5's -- wrong board variant selected");
static_assert(BOARD_SDIO_ESP_HOSTED_D1 == TAB5_C6_SDIO_D1_GPIO,
              "esp-hosted D1 pin is not the Tab5's -- wrong board variant selected");
static_assert(BOARD_SDIO_ESP_HOSTED_D2 == TAB5_C6_SDIO_D2_GPIO,
              "esp-hosted D2 pin is not the Tab5's -- wrong board variant selected");
static_assert(BOARD_SDIO_ESP_HOSTED_D3 == TAB5_C6_SDIO_D3_GPIO,
              "esp-hosted D3 pin is not the Tab5's -- wrong board variant selected");
static_assert(BOARD_SDIO_ESP_HOSTED_RESET == TAB5_C6_RESET_GPIO,
              "esp-hosted RESET pin is not the Tab5's -- wrong board variant selected");

namespace {

enum class State : uint8_t { kUntried, kUp, kFailed };
State s_state = State::kUntried;

// Assert WLAN_PWR_EN so the ESP32-C6 actually has power.
//
// THIS IS A HARD PREREQUISITE, not an optimisation. On Tab5 the C6's supply is
// gated by P0 of the PI4IOE5V6408 IO-expander at I2C 0x44 -- not by any P4
// GPIO -- and it is off at reset. Without this, esp-hosted toggles the C6's
// reset line and clocks SDIO at a chip that is simply not powered, producing
// `sdmmc_init_ocr: send_op_cond returned 0x107` (ESP_ERR_TIMEOUT) forever.
// Correct SDIO pins alone are NOT sufficient; that was the second half of the
// crash-loop root cause. See pins_config.h TAB5_WLAN_PWR_EN_IOEXP_BIT.
bool c6_power_on() {
    // Idempotent w.r.t. TouchTab5::init(), which begins the same bus with the
    // same pins; repeated here so this module is self-contained and does not
    // silently depend on touch having been initialised first.
    Wire.begin(TAB5_INTERNAL_I2C_SDA_GPIO, TAB5_INTERNAL_I2C_SCL_GPIO);

    return tab5_ioexp::set_output(TAB5_PWR_IOEXP_I2C_ADDR,
                                  TAB5_WLAN_PWR_EN_IOEXP_BIT, true);
}

}  // namespace

namespace hosted_link {

bool begin() {
    if (s_state != State::kUntried) {
        return s_state == State::kUp;
    }
    // Latch FAILED up front. Every early return below is a failure path, and
    // pre-latching means a future edit that adds one can't accidentally leave
    // the state at kUntried and re-arm the retry storm this module exists to
    // prevent. The success path overwrites it at the very end.
    s_state = State::kFailed;

    // Belt-and-braces over the compile-time fix in
    // boards/variants/quarky_tab5_p4/pins_arduino.h. WiFi.setPins() forwards to
    // hostedSetPins(), which mutates the same static sdio_pin_config_t that
    // esp32-hal-hosted.c initialises from the BOARD_SDIO_ESP_HOSTED_* macros --
    // so with the correct variant in place this is a no-op that sets the pins
    // to the values they already have. It is here because it is CHEAP INSURANCE
    // against the one way the variant fix can silently regress: if someone
    // later points `board` back at a stock definition (or PlatformIO fails to
    // pick up boards/ for any reason), the macros quietly revert to the generic
    // EV-board's wrong pins and the crash-loop returns with no compile error to
    // warn anyone. This call makes the Tab5 pin values appear in application
    // source, where they are greppable and cannot be silently overridden by a
    // board-selection change. It must run before any WiFi/BLE API call --
    // hostedSetPins() refuses (and logs) once esp-hosted is initialised.
    if (!WiFi.setPins(TAB5_C6_SDIO_CLK_GPIO, TAB5_C6_SDIO_CMD_GPIO,
                      TAB5_C6_SDIO_D0_GPIO, TAB5_C6_SDIO_D1_GPIO,
                      TAB5_C6_SDIO_D2_GPIO, TAB5_C6_SDIO_D3_GPIO,
                      TAB5_C6_RESET_GPIO)) {
        Serial.println("quarky-tab5: hosted_link FAILED to set C6 SDIO pins");
        return false;
    }

    if (!c6_power_on()) {
        Serial.println("quarky-tab5: hosted_link FAILED to assert WLAN_PWR_EN "
                       "on the 0x44 IO-expander -- C6 has no power");
        return false;
    }
    // Let the C6's rail come up and the chip finish its own power-on reset
    // before esp-hosted starts toggling its reset line and clocking SDIO.
    delay(100);

    Serial.printf("quarky-tab5: hosted_link bringing up ESP32-C6 over SDIO "
                  "(clk=%d cmd=%d d0=%d d1=%d d2=%d d3=%d rst=%d)...\n",
                  TAB5_C6_SDIO_CLK_GPIO, TAB5_C6_SDIO_CMD_GPIO,
                  TAB5_C6_SDIO_D0_GPIO, TAB5_C6_SDIO_D1_GPIO,
                  TAB5_C6_SDIO_D2_GPIO, TAB5_C6_SDIO_D3_GPIO,
                  TAB5_C6_RESET_GPIO);

    // hostedInitWiFi() is the same entry point WiFiGenericClass::wifiLowLevelInit()
    // uses, so calling it here does not duplicate work: it runs the SDIO card
    // init + slave handshake once, and every later WiFi call finds it already
    // initialised and returns immediately. Doing it eagerly (rather than
    // implicitly, on the first WiFi.mode()) is the whole point -- it gives us
    // one well-defined place where a C6 failure is observed, logged, and
    // latched, instead of N scattered places that each silently retry.
    if (!hostedInitWiFi()) {
        Serial.println("quarky-tab5: hosted_link FAILED -- ESP32-C6 link is "
                       "down; WiFi and BLE will be DISABLED for this boot. "
                       "Display/touch/UI continue normally.");
        return false;
    }

    // LIVENESS PROBE -- hostedInitWiFi() returning true is NOT proof the link
    // works. Confirmed on hardware: with the C6 unpowered, esp_hosted_init()
    // and esp_hosted_connect_to_slave() both still returned ESP_OK (the
    // transport layer defers/retries internally and only logs
    // `_CHECK_WITHOUT_ABORT ... ensure_slave_bus_ready failed`), so
    // hostedInitWiFi() reported success on a completely dead bus. Everything
    // downstream then walked into a link that wasn't there -- and NimBLE's
    // ble_transport_ll_init() wraps its transport bring-up in ESP_ERROR_CHECK,
    // so it did not fail gracefully, it called abort() and rebooted the board.
    //
    // hostedInit() calls hostedHasUpdate(), which populates the cached slave
    // firmware version via an actual RPC round-trip to the co-processor. A
    // version of 0.0.0 therefore means "the C6 never answered" -- a real
    // end-to-end check of the link, not just of the local init calls.
    uint32_t hmaj = 0, hmin = 0, hpat = 0, smaj = 0, smin = 0, spat = 0;
    hostedGetHostVersion(&hmaj, &hmin, &hpat);
    hostedGetSlaveVersion(&smaj, &smin, &spat);
    if (smaj == 0 && smin == 0 && spat == 0) {
        Serial.println("quarky-tab5: hosted_link FAILED -- ESP32-C6 did not "
                       "answer (slave firmware version 0.0.0); WiFi and BLE "
                       "DISABLED for this boot. Display/touch/UI continue "
                       "normally.");
        return false;
    }

    Serial.printf("quarky-tab5: hosted_link UP (host esp-hosted %u.%u.%u, "
                  "co-processor %u.%u.%u)\n",
                  (unsigned)hmaj, (unsigned)hmin, (unsigned)hpat,
                  (unsigned)smaj, (unsigned)smin, (unsigned)spat);

    s_state = State::kUp;
    return true;
}

bool available() {
    return s_state == State::kUp;
}

}  // namespace hosted_link
