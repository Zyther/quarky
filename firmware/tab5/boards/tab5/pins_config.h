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

// PANEL GEOMETRY. The Tab5's DSI panel is NATIVELY PORTRAIT: 720 pixels wide by
// 1280 tall. This is not a guess -- it is what the espp BSP configures the DPI
// timing with (`dpi_cfg.video_timing.h_size = display_width_` /
// `v_size = display_height_`, where m5stack-tab5.hpp declares
// `display_width_ = 720; display_height_ = 1280;`), and it matches the ILI9881C
// silicon, which is a 720 x 1280 driver. The DSI/DPI hardware and
// esp_lcd_panel_draw_bitmap() therefore work in 720x1280 coordinates ALWAYS.
#define TAB5_PANEL_NATIVE_WIDTH   720
#define TAB5_PANEL_NATIVE_HEIGHT  1280

// LOGICAL (LVGL-facing) ORIENTATION. The Tab5 is physically a landscape
// device, so DisplayTab5 presents a 1280x720 landscape surface to LVGL and
// rotates each flushed block into the panel's native portrait space itself
// (hardware PPA, with a software fallback) -- see src/hal/display_tab5.cpp.
//
//   90  = rotate the logical landscape image 90 degrees counter-clockwise
//         into native portrait (default)
//   270 = rotate 90 degrees clockwise instead -- use this if the picture comes
//         up upside down relative to what you expect
//   0   = no rotation; LVGL gets the raw native 720x1280 portrait surface.
//         This is exactly what the espp BSP does by default
//         (`rotation = DisplayRotation::LANDSCAPE`, which is enum value 0 and
//         is passed straight to lv_display_set_rotation as
//         LV_DISPLAY_ROTATION_0). Set this to bisect a bad picture: if 0 shows
//         a correct-but-sideways UI, the panel bring-up is fine and only the
//         rotation constant is wrong.
//
// WHICH WAY IS "UP" COULD NOT BE VERIFIED FROM SOURCE. Nothing in the BSP
// records how the panel is physically mounted in the Tab5's case, and the
// espp default (rotation 0) sidesteps the question by presenting portrait. 90
// vs 270 is therefore the one display value in this file that is a choice
// rather than a citation; flipping it is a one-line change.
#ifndef TAB5_DISPLAY_ROTATION
#define TAB5_DISPLAY_ROTATION 90
#endif

#if TAB5_DISPLAY_ROTATION == 90 || TAB5_DISPLAY_ROTATION == 270
#define TAB5_DISP_WIDTH   TAB5_PANEL_NATIVE_HEIGHT  // 1280
#define TAB5_DISP_HEIGHT  TAB5_PANEL_NATIVE_WIDTH   // 720
#elif TAB5_DISPLAY_ROTATION == 0
#define TAB5_DISP_WIDTH   TAB5_PANEL_NATIVE_WIDTH   // 720
#define TAB5_DISP_HEIGHT  TAB5_PANEL_NATIVE_HEIGHT  // 1280
#else
#error "TAB5_DISPLAY_ROTATION must be 0, 90 or 270"
#endif

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
// IMPLEMENTED: src/hal/display_tab5.cpp drives this via hal/io_expander.h
// (assert / 10 ms / release / 120 ms, per video.cpp initialize_lcd()).
// This matters more than it looks: at power-on the PI4IOE5V6408 leaves every
// pin high-impedance, so until something drives P4 the panel's reset line is
// simply floating and the controller may never come out of reset -- which is
// consistent with the pre-fix I2C scan finding NO touch controller at all
// (neither 0x14 nor 0x55) on the physical unit.
#define TAB5_DISP_RST_GPIO  -1 // no raw reset GPIO exists; see comment above
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

// Panel IC init command list: NOW TRANSCRIBED. Task 5's TODO here said these
// lived in "separate espp components" and could not be found; they are not
// top-level `ili9881`/`st7123` components (which is why that search failed) --
// both classes live inside espp's shared `display_drivers` component:
//   components/display_drivers/include/ili9881.hpp  (espp::Ili9881, 202 cmds)
//   components/display_drivers/include/st7123.hpp   (espp::St7123,   28 cmds)
// Mechanically transcribed into src/hal/tab5_panel_cmds.h; see that file's
// header for the full citation and the round-trip verification method.

// Runtime panel-controller detection, transcribed from video.cpp
// `M5StackTab5::detect_display_controller()`:
//   1. probe I2C 0x14 -- a GT911 answering there means this is the ILI9881
//      hardware revision (standalone touch IC + ILI9881 display driver)
//   2. else probe I2C 0x55 -- the ST7123 revision (TDDI part, touch built in)
//   3. else UNKNOWN (the BSP gives up; see display_tab5.cpp for what this
//      project does instead, and why)
// Both probes must happen AFTER the LCD reset pulse above, exactly as the BSP
// orders them -- the ST7123 cannot answer on I2C while it is held in reset.
#define TAB5_ILI9881_PROBE_I2C_ADDR  0x14  // GT911 presence => ILI9881 variant
#define TAB5_ST7123_I2C_ADDR         0x55  // ST7123 TDDI controller, probed directly

// DPI (parallel-in, DSI-out) video timing, confirmed from video.cpp. These are
// consumed by src/hal/display_tab5.cpp when it builds the DPI panel. The
// h_size/v_size that go with them are the NATIVE portrait 720x1280 above.
#define TAB5_DPI_CLK_MHZ_ILI9881        60
#define TAB5_DPI_HSYNC_BP_ILI9881       140
#define TAB5_DPI_HSYNC_PW_ILI9881       40
#define TAB5_DPI_HSYNC_FP_ILI9881       40
#define TAB5_DPI_VSYNC_BP_ILI9881       20
#define TAB5_DPI_VSYNC_PW_ILI9881       4
#define TAB5_DPI_VSYNC_FP_ILI9881       20
// DO NOT RAISE the ST7123 pixel clock above 70 MHz. Per video.cpp's own
// comment: the ST7123 is a TDDI part whose touch engine scans during the
// display blanking interval, timed against the pixel clock its vendor init
// table was tuned for; running faster shrinks the blanking window and desyncs
// the touch scan (panel shows, touch never reports).
#define TAB5_DPI_CLK_MHZ_ST7123         70
#define TAB5_DPI_HSYNC_BP_ST7123        40
#define TAB5_DPI_HSYNC_PW_ST7123        2
#define TAB5_DPI_HSYNC_FP_ST7123        40
#define TAB5_DPI_VSYNC_BP_ST7123        8
#define TAB5_DPI_VSYNC_PW_ST7123        2
#define TAB5_DPI_VSYNC_FP_ST7123        220

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
// Independently corroborated by the pioarduino framework's own
// variants/m5stack_tab5/pins_arduino.h, which declares the identical seven
// values.
//
// STATUS (updated by the C6 SDIO hotfix, see
// hotfix-c6-sdio-pins-report.md): these values are now LIVE in the build.
// Task 10 recorded them here as documentation only, and flagged that the
// then-selected `esp32-p4-evboard` board baked in the generic EV-board's
// wrong C6 pins instead (see TAB5_EVBOARD_C6_* below) -- which on real
// hardware meant esp-hosted drove the wrong GPIOs, the C6 never enumerated,
// and the board crash-looped into the brownout detector on every boot. That
// is fixed two ways now:
//   1. At compile time, by this repo's own board + variant
//      (boards/quarky_tab5_p4.json, boards/variants/quarky_tab5_p4/
//      pins_arduino.h), which set BOARD_SDIO_ESP_HOSTED_* to these values.
//      A `-D` build flag could NOT have done this: the framework's macros
//      are unconditional #defines with no #ifndef guard.
//   2. At run time, by src/hal/hosted_link.cpp, which passes the macros
//      below to WiFi.setPins() before any radio API call.
#define TAB5_C6_SDIO_CLK_GPIO   12
#define TAB5_C6_SDIO_CMD_GPIO   13
#define TAB5_C6_SDIO_D0_GPIO    11
#define TAB5_C6_SDIO_D1_GPIO    10
#define TAB5_C6_SDIO_D2_GPIO    9
#define TAB5_C6_SDIO_D3_GPIO    8
#define TAB5_C6_RESET_GPIO      15
#define TAB5_C6_SDIO_SDMMC_SLOT 1

// ESP32-C6 POWER RAIL -- the C6 is NOT powered by default.
// Found the hard way during the C6 SDIO hotfix: even with the correct SDIO
// pins above, `sdmmc_init_ocr: send_op_cond returned 0x107` (ESP_ERR_TIMEOUT)
// on every attempt, because the co-processor had no power at all. On Tab5 the
// C6's supply is gated by WLAN_PWR_EN, which is NOT a P4 GPIO -- it is P0 of
// the *second* PI4IOE5V6408 I2C IO-expander, at address 0x44 (distinct from
// the 0x43 expander that carries LCD_RST/TP_RST above), on the same internal
// I2C bus (TAB5_INTERNAL_I2C_SDA_GPIO/SCL_GPIO).
// Source: espp/m5stack-tab5 m5stack-tab5.hpp --
//   `static constexpr int WLAN_PWR_EN_PIN = (1 << 0); // WLAN_PWR_EN`
//   and `IOX_0x44_DEFAULT_OUTPUTS = WLAN_PWR_EN_PIN | USB_5V_EN_PIN`
// (the BSP asserts it as part of expander init, which is why BSP-based
// projects never notice it is a prerequisite). Driven by
// src/hal/hosted_link.cpp before esp-hosted bring-up.
#define TAB5_PWR_IOEXP_I2C_ADDR   0x44
#define TAB5_WLAN_PWR_EN_IOEXP_BIT 0    // PI4IOE5V6408 P0 / WLAN_PWR_EN

// PI4IOE5V6408 register map (both the 0x43 and 0x44 expanders are the same
// part). Single-byte register addresses -- note this differs from the GT911's
// 16-bit big-endian register addressing used in touch_gt911.cpp.
// Source: espp/components/pi4ioe5v/include/pi4ioe5v.hpp register enum.
#define PI4IOE5V6408_REG_CHIP_ID_CTRL   0x01
#define PI4IOE5V6408_REG_DIRECTION      0x03  // 1 = output, 0 = input
#define PI4IOE5V6408_REG_OUTPUT         0x05
#define PI4IOE5V6408_REG_OUTPUT_HIGH_IM 0x07  // 1 = high-Z (output disconnected)
#define PI4IOE5V6408_REG_INPUT_DEFAULT  0x09
#define PI4IOE5V6408_REG_PULL_ENABLE    0x0B
#define PI4IOE5V6408_REG_PULL_SELECT    0x0D
#define PI4IOE5V6408_REG_INPUT          0x0F

// Generic ESP32-P4-Function-EV-Board's C6 pins, kept for reference and as a
// regression tripwire: these are the WRONG-for-Tab5 values that
// framework-arduinoespressif32/variants/esp32p4/pins_arduino.h bakes in, and
// that this project shipped until the C6 SDIO hotfix. If a future boot ever
// logs esp-hosted using these numbers again, the custom board definition has
// been bypassed -- see hotfix-c6-sdio-pins-report.md.
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
