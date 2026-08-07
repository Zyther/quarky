#pragma once
// M5Stack Tab5 board pin/config constants for the MIPI-DSI display bring-up.
//
// SOURCE: espp/m5stack-tab5 BSP (esp-cpp/espp repo), commit on `main` branch,
// fetched 2026-08-07 via `gh api repos/esp-cpp/espp/contents/...`:
//   https://github.com/esp-cpp/espp/blob/main/components/m5stack-tab5/include/m5stack-tab5.hpp
//   https://github.com/esp-cpp/espp/blob/main/components/m5stack-tab5/src/video.cpp
//   https://github.com/esp-cpp/espp/blob/main/components/m5stack-tab5/src/m5stack-tab5.cpp
// Component registry page (did not expose source directly, used to confirm
// component identity/version): https://components.espressif.com/components/espp/m5stack-tab5
//
// NOTE ON HARDWARE VARIANTS: the espp BSP auto-detects between two possible
// panel controller ICs at runtime via I2C probe — earlier Tab5 revisions ship
// an ILI9881 (detected indirectly via a GT911 touch controller at I2C 0x14),
// newer revisions ship an ST7123 (probed directly at I2C 0x55, a TDDI part
// combining touch + display). Which one a given physical unit has cannot be
// determined from source research alone; it must be probed at runtime.
// See video.cpp: M5StackTab5::detect_display_controller().

#define TAB5_DISP_WIDTH   1280
#define TAB5_DISP_HEIGHT  720

// Backlight: real GPIO, confirmed. In the espp BSP this is driven via LEDC
// PWM (5 kHz, 10-bit duty, channel 0/timer 0) for brightness control, not a
// plain digital pin -- labeled "LEDA" on the schematic net name in the BSP.
// A simple digitalWrite(HIGH) (as used by this task's smoke test) drives it
// to full brightness but skips PWM dimming; that's acceptable for bring-up.
// Source: m5stack-tab5.hpp line ~681 `lcd_backlight_io = GPIO_NUM_22`.
#define TAB5_DISP_BL_GPIO   22

// Panel reset: NOT a raw ESP32-P4 GPIO. The espp BSP routes LCD_RST through
// an on-board PI4IOE5V6408 I2C IO-expander at I2C address 0x43, output bit
// P4 (IO43_BIT_LCD_RST = 4 / LCD_RST_PIN = (1 << 4)), reached over the
// internal I2C bus (SDA = GPIO_NUM_31, SCL = GPIO_NUM_32). There is no
// direct-GPIO reset line to assign here.
// Source: m5stack-tab5.hpp lines ~623-624, ~638, ~628; m5stack-tab5.cpp
// `M5StackTab5::lcd_reset()` (calls set_io_expander_output(0x43, 4, ...)).
// TODO: this task's DisplayTab5 does not yet drive panel reset (the brief's
// init() only toggles backlight), so no I2C IO-expander driver exists here
// yet. When panel reset is implemented, it needs an I2C IO-expander client
// (PI4IOE5V6408 protocol) targeting the address/bit below, not a GPIO write.
#define TAB5_DISP_RST_GPIO  -1 // TODO: no raw reset GPIO exists; see comment above
#define TAB5_DISP_RST_IOEXP_I2C_ADDR 0x43
#define TAB5_DISP_RST_IOEXP_BIT      4     // IO43_BIT_LCD_RST / PI4IOE5V6408 P4
#define TAB5_INTERNAL_I2C_SDA_GPIO   31
#define TAB5_INTERNAL_I2C_SCL_GPIO   32

// MIPI-DSI bus configuration (esp_lcd_dsi_bus_config_t), confirmed from
// video.cpp `M5StackTab5::initialize_lcd()`:
#define TAB5_DSI_NUM_DATA_LANES      2
// Lane bit rate depends on which panel controller is detected at runtime:
//   ILI9881 -> 730 Mbps, ST7123 -> 965 Mbps. Not a single fixed constant.
#define TAB5_DSI_LANE_BIT_RATE_MBPS_ILI9881 730
#define TAB5_DSI_LANE_BIT_RATE_MBPS_ST7123  965

// ESP32-P4 MIPI-DSI PHY requires an LDO power rail before the DSI bus is
// created (esp_ldo_acquire_channel), confirmed from video.cpp:
#define TAB5_DSI_PHY_PWR_LDO_CHANNEL      3
#define TAB5_DSI_PHY_PWR_LDO_VOLTAGE_MV   2500

// Panel IC init command list: genuinely NOT transcribed here. In the espp
// BSP this lives inside separate `espp::Ili9881` / `espp::St7123` display
// driver classes (their own components, not inlined in m5stack-tab5's
// video.cpp), which construct the vendor init command tables internally and
// expose only a high-level `initialize()` call
// (see video.cpp calls to `std::make_shared<espp::Ili9881>(...)` and
// `std::make_shared<espp::St7123>(...)`). Locating and transcribing those
// per-controller command byte tables was out of scope for the research pass
// backing this file (would require fetching two more component sources);
// flagged here explicitly rather than fabricated.
// TODO: fetch espp's `ili9881` and `st7123` display-driver components
// (github.com/esp-cpp/espp, likely under components/display_drivers or a
// dedicated ili9881/st7123 component) for the actual init command sequences
// before implementing a real (non-smoke-test) panel bring-up.

// DPI (parallel-in, DSI-out) video timing, confirmed from video.cpp for
// reference/documentation -- not yet consumed by DisplayTab5 in this task
// (the brief's init()/flush() stubs don't construct a DPI panel):
//   ILI9881: dpi_clock_freq_mhz=60, hsync bp/pw/fp=140/40/40,
//            vsync bp/pw/fp=20/4/20
//   ST7123:  dpi_clock_freq_mhz=70 (do not increase -- desyncs the ST7123's
//            touch scan per video.cpp comment), hsync bp/pw/fp=40/2/40,
//            vsync bp/pw/fp=8/2/220
