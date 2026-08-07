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

// ---------------------------------------------------------------------------
// GT911 touch controller (Task 6)
//
// SOURCE: espp/m5stack-tab5 BSP (esp-cpp/espp repo, `main` branch), fetched
// 2026-08-07 via `curl raw.githubusercontent.com` (component source is public,
// no PAT needed for raw fetch; `gh api` used only to list the gt911 component
// directory):
//   https://github.com/esp-cpp/espp/blob/main/components/m5stack-tab5/include/m5stack-tab5.hpp
//   https://github.com/esp-cpp/espp/blob/main/components/m5stack-tab5/src/m5stack-tab5.cpp
//   https://github.com/esp-cpp/espp/blob/main/components/m5stack-tab5/src/touchpad.cpp
//   https://github.com/esp-cpp/espp/blob/main/components/m5stack-tab5/src/video.cpp
//   https://github.com/esp-cpp/espp/blob/main/components/gt911/include/gt911.hpp
//
// Per pins_config.h's earlier variant note: this covers the ILI9881-variant
// Tab5, which uses a *standalone* GT911 touch IC. The newer ST7123-variant
// Tab5 uses the ST7123's *integrated* touch engine instead (a different I2C
// address/driver entirely, `espp::St7123Touch` -- not a GT911, out of scope
// for TouchGT911). Which variant a given unit has cannot be determined from
// source research; must be probed at runtime (same caveat as the display).

// GT911 shares the same internal I2C bus as the display's IO-expander-routed
// reset (TAB5_INTERNAL_I2C_SDA_GPIO / TAB5_INTERNAL_I2C_SCL_GPIO above,
// GPIO 31/32, I2C_NUM_0 in the BSP). Confirmed: m5stack-tab5.hpp lines
// ~619-624 ("Internal I2C (GT911 touch, ES8388/ES7210 audio, BMI270 IMU,
// RX8130CE RTC, INA226 power, PI4IOE5V6408 IO expanders)").

// I2C address: the GT911 datasheet defines two possible addresses selected by
// the state of its own INT pin at power-on (0x5D if INT held low, 0x14 if
// held high). The espp BSP hardwires Tab5 to the 0x14 variant -- confirmed by
// TWO independent sources agreeing:
//   1. touchpad.cpp `initialize_touch()`, GT911 branch: `.address =
//      TouchDriver::DEFAULT_ADDRESS_2, // GT911 0x14` (explicit inline
//      comment in the BSP source itself).
//   2. video.cpp `detect_display_controller()`: probes for the ILI9881
//      variant via `i2c.probe_device(0x14)` -- i.e. "does a GT911 answer at
//      0x14" is literally how the BSP tells the two hardware revisions apart.
// This directly corrects the task brief's own placeholder guess of 0x5D.
#define TAB5_TOUCH_I2C_ADDR   0x14

// Touch interrupt: real GPIO, confirmed. GT911 drives this line low when new
// touch data is ready (edge-triggered in the BSP, though polling the status
// register works too and is what this task's TouchGT911 does).
// Source: m5stack-tab5.hpp line ~682 `touch_interrupt_io = GPIO_NUM_23;
// // TP_INT`; touchpad.cpp comment "The GT911 drives TP_INT, so read it from
// the touch interrupt."
#define TAB5_TOUCH_INT_GPIO   23

// Touch reset: like LCD_RST, NOT a raw ESP32-P4 GPIO -- routed through the
// same PI4IOE5V6408 I2C IO-expander at 0x43 used for LCD_RST, but a
// different output bit: P5 (IO43_BIT_TP_RST = 5 / TP_RST_PIN = (1 << 5)),
// active-low (BSP's `touch_reset(true)` == assert == drive P5 low).
// Source: m5stack-tab5.hpp lines ~629, ~638; m5stack-tab5.cpp
// `M5StackTab5::touch_reset()` (calls set_io_expander_output(0x43, 5, ...));
// touchpad.cpp GT911 branch: reset sequence is assert, delay(10ms),
// release, delay(50ms) before first touch read.
// TODO: same situation as TAB5_DISP_RST_GPIO above -- this task's
// TouchGT911 does not yet drive touch reset (the BSP's `IoExpander`/
// PI4IOE5V6408 register protocol was not transcribed in this research pass,
// same scope cut as the display's IO-expander reset). GT911 typically still
// answers I2C without an explicit reset pulse in many bring-ups (power-on
// reset default), but a implementer hitting "touch not responding" on real
// hardware should implement this IO-expander reset pulse first before
// suspecting the I2C address/registers.
#define TAB5_TOUCH_RST_GPIO -1 // TODO: no raw reset GPIO exists; see comment above
#define TAB5_TOUCH_RST_IOEXP_I2C_ADDR 0x43
#define TAB5_TOUCH_RST_IOEXP_BIT      5     // IO43_BIT_TP_RST / PI4IOE5V6408 P5

// GT911 register map + wire protocol, confirmed from
// components/gt911/include/gt911.hpp (`espp::Gt911`), whose own comment
// attributes it to Espressif's esp-bsp `esp_lcd_touch_gt911.c`
// (github.com/espressif/esp-bsp/blob/master/components/lcd_touch/esp_lcd_touch_gt911/esp_lcd_touch_gt911.c):
//   - Register address is 2 bytes, sent big-endian (MSB first) -- confirmed
//     from base_peripheral.hpp `put_register_bytes()`, the espp I2C register
//     helper the Gt911 driver is built on.
//   - Status/point-count register: 0x814E. Bit 7 = data ready, bit 4 = only
//     the home key was pressed (read key register instead), low nibble =
//     touch point count (0-5) when bit 7 set and bit 4 clear.
//   - First touch point registers start at 0x814F (POINTS/POINT_1); each
//     point (POINT_2=0x8157, POINT_3=0x815F, ... 8 bytes apart) is packed
//     {uint8_t track_id; uint16_t x; uint16_t y; uint16_t area; uint8_t
//     reserved;} with x/y/area little-endian on the wire (only the 2-byte
//     *register address* is big-endian; point payload bytes are not).
//   - After reading, write 0x00 back to 0x814E as a sync/clear signal so the
//     chip knows the host consumed the report.
#define TAB5_TOUCH_REG_STATUS   0x814E
#define TAB5_TOUCH_REG_POINT_1  0x814F
#define TAB5_TOUCH_CONTACT_SIZE 8 // bytes per touch point record

// ---------------------------------------------------------------------------
// microSD (SDIO) and ESP32-C6 SDIO-host-sharing research (Task 10)
//
// SOURCE: espp/m5stack-tab5 BSP (esp-cpp/espp repo, `main` branch), fetched
// 2026-08-07 via `curl raw.githubusercontent.com`:
//   https://github.com/esp-cpp/espp/blob/main/components/m5stack-tab5/include/m5stack-tab5.hpp
//   https://github.com/esp-cpp/espp/blob/main/components/m5stack-tab5/src/sdcard.cpp
// Cross-checked against ESP-IDF ESP32-P4 headers installed by this project's
// pioarduino toolchain:
//   framework-arduinoespressif32-libs/esp32p4/include/soc/esp32p4/include/soc/sdmmc_pins.h
//   framework-arduinoespressif32-libs/esp32p4/include/esp_driver_sdmmc/include/driver/sdmmc_host.h
//   framework-arduinoespressif32-libs/esp32p4/qio_qspi/include/sdkconfig.h
// and against the Arduino core variant actually selected by this project's
// platformio.ini (`board = esp32-p4-evboard` -> variant `esp32p4`):
//   framework-arduinoespressif32/variants/esp32p4/pins_arduino.h
//   framework-arduinoespressif32/cores/esp32/esp32-hal-hosted.c
//
// FINDING: the microSD card and the ESP32-C6 co-processor are on two
// separate hardware SDMMC/SDIO slots of the ESP32-P4 (slot 0 vs slot 1),
// with disjoint GPIO pins on real Tab5 hardware -- no shared bus wire, no
// electrical contention. This resolves the foundation spec's flagged risk
// from source. Residual caveats (indirect DMA/interrupt-level resource
// sharing under heavy concurrent load; a real pins_arduino.h mismatch
// between this project's generic EV-board pin config and Tab5's actual C6
// pins) are documented in storage_sd.cpp and task-10-report.md.

// microSD, real Tab5 pins, on SDMMC host SLOT 0 (fixed IOMUX -- these exact
// GPIOs are hardwired in silicon for slot 0 on the ESP32-P4, confirmed
// identical to SDMMC_SLOT0_IOMUX_PIN_NUM_* in soc/sdmmc_pins.h, which is why
// they match this project's generic EV-board pins_arduino.h defaults too).
#define TAB5_SD_CLK_GPIO   43
#define TAB5_SD_CMD_GPIO   44
#define TAB5_SD_D0_GPIO    39
#define TAB5_SD_D1_GPIO    40
#define TAB5_SD_D2_GPIO    41
#define TAB5_SD_D3_GPIO    42
#define TAB5_SD_SDMMC_SLOT 0

// ESP32-C6 co-processor link, real Tab5 pins, on SDMMC host SLOT 1
// (GPIO-matrix-routed -- confirmed disjoint from the SD pins above).
// Source: m5stack-tab5.hpp, comments label these net names "SDIO2_*".
// NOTE: this project's currently-selected PlatformIO board
// (`esp32-p4-evboard`) does NOT use these values -- its pins_arduino.h
// hardcodes the *generic* ESP32-P4-Function-EV-Board's C6 pins instead
// (18/19/14/15/16/17/54, see BOARD_SDIO_ESP_HOSTED_* below), consumed by
// cores/esp32/esp32-hal-hosted.c at compile time. That is a real,
// unresolved pin mismatch against real Tab5 hardware, flagged forward in
// task-10-report.md -- not fixed here (board-config decision, affects
// Task 9's radio too, out of this task's scope).
#define TAB5_C6_SDIO_CLK_GPIO   12
#define TAB5_C6_SDIO_CMD_GPIO   13
#define TAB5_C6_SDIO_D0_GPIO    11
#define TAB5_C6_SDIO_D1_GPIO    10
#define TAB5_C6_SDIO_D2_GPIO    9
#define TAB5_C6_SDIO_D3_GPIO    8
#define TAB5_C6_RESET_GPIO      15
#define TAB5_C6_SDIO_SDMMC_SLOT 1

// Generic ESP32-P4-Function-EV-Board's C6 pins, for reference -- these are
// what's ACTUALLY baked into this project's build today via
// framework-arduinoespressif32/variants/esp32p4/pins_arduino.h
// (BOARD_HAS_SDIO_ESP_HOSTED / BOARD_SDIO_ESP_HOSTED_*), not the Tab5
// values above. See caveat (b) in storage_sd.cpp.
#define TAB5_EVBOARD_C6_SDIO_CLK_GPIO   18
#define TAB5_EVBOARD_C6_SDIO_CMD_GPIO   19
#define TAB5_EVBOARD_C6_SDIO_D0_GPIO    14
#define TAB5_EVBOARD_C6_SDIO_D1_GPIO    15
#define TAB5_EVBOARD_C6_SDIO_D2_GPIO    16
#define TAB5_EVBOARD_C6_SDIO_D3_GPIO    17
#define TAB5_EVBOARD_C6_RESET_GPIO      54

// SD card power-enable: the generic EV-board's pins_arduino.h also defines
// BOARD_SDMMC_POWER_CHANNEL=4 / BOARD_SDMMC_POWER_PIN=45 (active LOW),
// which SD_MMC.begin() will toggle by default. The Tab5 BSP source does not
// document an equivalent dedicated SD power-switch GPIO -- genuinely
// unconfirmed whether Tab5's SD socket needs/has one. TODO: verify against
// Tab5's actual schematic before relying on this in production; left as the
// EV-board default for this bring-up pass rather than fabricated.
