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
// Lane bit rate depends on which panel controller is detected at runtime.
//
// CORRECTED 2026-08-08 from M5Stack's own M5GFX library (the vendor's driver
// for this exact board), src/M5GFX.cpp board_M5Tab5 bring-up:
//     bus_cfg.lane_mbps = hit_st7121 ? 900 : 1040;
// espp used 965 Mbps for "ST7123", which matches neither vendor value and is
// not traceable to any M5Stack source. See the ST7121/ST7123 note below.
#define TAB5_DSI_LANE_BIT_RATE_MBPS_ST7123  1040
#define TAB5_DSI_LANE_BIT_RATE_MBPS_ST7121  900
// ILI9881: left on espp's value deliberately. M5GFX uses 1040 Mbps / 80 MHz
// for this path too, but this project pairs it with espp's 202-command
// ILI9881 init table, and espp's table was written against espp's 730 Mbps /
// 60 MHz. Mixing one vendor's table with the other's clocks is a combination
// neither project ships. This unit is not an ILI9881 (no GT911 at 0x14), so
// the path cannot be tested here; keeping espp's self-consistent pair is the
// lower-risk choice. If an ILI9881 Tab5 ever turns up and is blank, try the
// vendor pair (1040 Mbps / 80 MHz) together with M5GFX's Panel_ILI9881C table.
#define TAB5_DSI_LANE_BIT_RATE_MBPS_ILI9881 730

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
#define TAB5_ST7123_I2C_ADDR         0x55  // Sitronix TDDI controller, probed directly

// CORRECTION 2026-08-08 -- "something ACKs at 0x55" IS NOT ENOUGH.
//
// espp's detect_display_controller() treats an ACK at 0x55 as "this is an
// ST7123". M5Stack's own M5GFX library shows that is ambiguous: the Tab5 ships
// with one of TWO different Sitronix TDDI panels, ST7121 or ST7123, and BOTH
// live at 0x55 (M5GFX: `Touch_ST7123::default_addr = 0x55`, used for both).
// They need different register tables, different DSI lane rates and different
// DPI timing, so guessing wrong gives a panel that ACKs every command over DSI
// and never lights up.
//
// The vendor discriminates by reading the touch firmware version -- a 1-byte
// read from 16-bit register 0x0000 at 0x55 (M5GFX src/M5GFX.cpp):
//     uint8_t fw_reg[2] = { 0, 0 };
//     transactionWriteRead(port, Touch_ST7123::default_addr, fw_reg, 2, &fw_version, 1, ...);
//     if (fw_version == 1) { hit_st7121 = true; }
//     if (fw_version == 3) { hit_st7123 = true; }
// retried up to 3 times with 10 ms between attempts.
#define TAB5_ST_FW_VERSION_REG_HI    0x00
#define TAB5_ST_FW_VERSION_REG_LO    0x00
#define TAB5_ST_FW_VERSION_ST7121    1
#define TAB5_ST_FW_VERSION_ST7123    3

// The vendor's secondary check is a DSI read of register 0xF4 (2 bytes),
// expecting 0x71 0x23. DELIBERATELY NOT IMPLEMENTED: esp_lcd_panel_io_rx_param()
// on a DBI channel lands in mipi_dsi_hal_host_gen_read_dcs_command(), whose
// read-FIFO wait loops (`while (mipi_dsi_host_ll_gen_is_read_cmd_busy(...));`
// and `while (mipi_dsi_host_ll_gen_is_read_fifo_empty(...));` in
// components/hal/mipi_dsi_hal.c, IDF v5.5) have NO timeout. A panel that does
// not answer hangs the boot with no serial output, which is strictly worse
// than a wrong guess. The I2C firmware-version read above is bounded and is
// the vendor's own primary discriminator anyway.

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
// Sitronix DPI timing. CORRECTED 2026-08-08 from M5Stack's own M5GFX library,
// src/M5GFX.cpp board_M5Tab5 bring-up (the `hit_st7123` / `hit_st7121`
// branches). espp had a single "ST7123" profile that turns out to be a blend:
// the ST7123's porches with the ST7121's 70 MHz pixel clock. Both variants are
// now carried separately and selected at runtime.
//
// The M5GFX source carries two of its own warnings about these numbers, kept
// here verbatim in spirit because they explain why they look asymmetric:
//   * ST7123 vsync: "back + pulse == 10. If it is out of sync, the display
//     position will shift vertically."
//   * ST7123 vsync front porch: "reducing the front porch will cause the touch
//     panel to stop working." (It is a TDDI part -- the touch engine scans in
//     the vertical blanking interval, so the blanking budget is load-bearing.)
// espp's "DO NOT RAISE above 70 MHz" comment was espp's own reasoning about
// that touch-scan constraint, not an M5Stack statement; the vendor runs the
// ST7123 at 80 MHz with a 220-line front porch.
#define TAB5_DPI_CLK_MHZ_ST7123         80
#define TAB5_DPI_HSYNC_BP_ST7123        40
#define TAB5_DPI_HSYNC_PW_ST7123        2
#define TAB5_DPI_HSYNC_FP_ST7123        40
#define TAB5_DPI_VSYNC_BP_ST7123        8
#define TAB5_DPI_VSYNC_PW_ST7123        2
#define TAB5_DPI_VSYNC_FP_ST7123        220
// ST7121: a DIFFERENT panel that also answers at I2C 0x55. espp does not know
// this variant exists. Note the much wider vsync pulse (20 vs 2).
#define TAB5_DPI_CLK_MHZ_ST7121         70
#define TAB5_DPI_HSYNC_BP_ST7121        40
#define TAB5_DPI_HSYNC_PW_ST7121        2
#define TAB5_DPI_HSYNC_FP_ST7121        40
#define TAB5_DPI_VSYNC_BP_ST7121        24
#define TAB5_DPI_VSYNC_PW_ST7121        20
#define TAB5_DPI_VSYNC_FP_ST7121        200

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
//
// RESOLVED 2026-08-08 (touch hotfix): that caveat was correct and THIS UNIT IS
// THE OTHER VARIANT. It is an ST7121, so it has no GT911 anywhere -- the
// GT911 macros below are retained only for ILI9881-revision boards. The real
// touch hardware for this unit is the TDDI block documented in the next
// section. See hotfix-touch-report.md.

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
// IMPLEMENTED: src/hal/touch_tab5.cpp drives this via hal/io_expander.h, but
// ONLY on the GT911 fallback path. It must NOT be pulsed for an ST7121/ST7123
// TDDI panel -- see the "DO NOT PULSE TP_RST" note in the ST touch section
// below. The old GT911-only driver pulsed it unconditionally.
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
// ST7121 / ST7123 INTEGRATED (TDDI) TOUCH -- the touch hardware on THIS unit
//
// These panels are TDDI parts: Touch and Display Driver Integration. There is
// no separate touch IC. The capacitive touch engine is inside the same silicon
// as the MIPI-DSI display controller and is read over I2C at the panel's own
// address, 0x55 -- the exact address DisplayTab5 already uses to read the
// firmware-version byte that identifies ST7121 vs ST7123.
//
// SOURCES (fetched 2026-08-08; two independent implementations that agree):
//  1. m5stack/M5GFX (the vendor's own library for this board):
//     src/lgfx/v1/platforms/esp32p4/Touch_ST7123.hpp
//       `static constexpr const uint8_t default_addr = 0x55;`
//     src/lgfx/v1/platforms/esp32p4/Touch_ST7123.cpp
//       -- every register constant below, verbatim, plus getTouchRaw().
//     src/M5GFX.cpp, board_M5Tab5 (L2778-2823): an ST7121 or ST7123 panel gets
//       `new Touch_ST7123()`; only the ILI9881C revision gets `new Touch_GT911()`.
//     src/M5GFX.cpp L2850-2861: touch cfg for Tab5 -- pin_rst = -1 (no touch
//       reset), pin_sda 31, pin_scl 32, pin_int 23, freq 400000,
//       x_max 719, y_max 1279, offset_rotation 0.
//  2. esp-cpp/espp components/st7123touch/include/st7123touch.hpp -- an
//     independent implementation of the same protocol. Agrees with M5GFX on
//     every register number, the 7-byte report stride, the with_coord bit and
//     the coordinate field packing.
//  3. docs.m5stack.com/en/core/Tab5 -- vendor documentation listing 0x55 as
//     "ST7123 or ST7121 (display/touch driver)" on the internal I2C bus, and
//     noting that units before 2025-10-14 used a GT911 at 0x14 instead.
//
// IMPORTANT -- DO NOT PULSE TP_RST FOR THIS PART. espp's st7123touch.hpp class
// documentation states: "The ST7123's touch engine is gated by the LCD reset
// (LCD_RST) line, NOT the TP_RST line used by standalone touch controllers
// such as the GT911. When used in a system that has a separate TP_RST signal
// (e.g. M5Stack Tab5), do NOT toggle TP_RST for this chip - doing so may knock
// the touch I2C endpoint offline." M5GFX agrees by setting pin_rst = -1.
// LCD_RST is already pulsed by DisplayTab5::init(), which is what brings the
// touch engine up.
#define TAB5_TOUCH_ST_I2C_ADDR        0x55

// Register map. 16-bit register pointer, big-endian, repeated-START read.
#define TAB5_TOUCH_ST_REG_FW_VERSION  0x0000 // 1 byte; 1 => ST7121, 3 => ST7123
#define TAB5_TOUCH_ST_REG_MAX_X_H     0x0005 // then _L 0x0006, Y_H 0x0007, Y_L 0x0008
#define TAB5_TOUCH_ST_REG_MAX_TOUCHES 0x0009 // 1 byte, firmware-configured slot count
#define TAB5_TOUCH_ST_REG_FW_REVISION 0x000C // 4 bytes
#define TAB5_TOUCH_ST_REG_ADV_INFO    0x0010 // 1 byte; see bit definitions below
#define TAB5_TOUCH_ST_REG_REPORT_0    0x0014 // max_touches x 7-byte reports

// ADV_INFO (0x0010) bit layout, from M5GFX's `adv_info_t` bitfield
// (reserved_0_1:2, with_prox:1, with_coord:1, prox_status:3, rst_chip:1 --
// LSB-first on this little-endian target). Only with_coord is used; espp
// spells the same bit as `ADV_INFO_WITH_COORD = (1 << 3)`.
#define TAB5_TOUCH_ST_ADV_WITH_COORD  (1 << 3)

// Touch report: 7 bytes per slot, from M5GFX's `touch_report_t`
// (x_h:6, reserved_6:1, valid:1 | x_l | y_h | y_l | area | intensity |
// reserved). So valid = byte[0] & 0x80,
// x = ((byte[0] & 0x3F) << 8) | byte[1], y = (byte[2] << 8) | byte[3].
#define TAB5_TOUCH_ST_REPORT_SIZE     7
#define TAB5_TOUCH_ST_MAX_POINTS      10 // M5GFX `max_touch_points = 10`

// Bus speed for touch reads. M5GFX uses 400 kHz for the Tab5 touch object.
#define TAB5_TOUCH_I2C_FREQ_HZ        400000

// Opt-in diagnostics, compiled out of normal builds.
//   -DTAB5_TOUCH_TRACE=1       log every press/release/move edge over serial
//   -DTAB5_TOUCH_I2C_CENSUS=1  print the labelled internal-I2C scan even on a
//                              successful boot (it is printed on failure
//                              regardless)
#ifndef TAB5_TOUCH_TRACE
#define TAB5_TOUCH_TRACE 0
#endif
#ifndef TAB5_TOUCH_I2C_CENSUS
#define TAB5_TOUCH_I2C_CENSUS 0
#endif

// ---------------------------------------------------------------------------
// Internal I2C bus census (GPIO 31/32), as observed on this unit and matched
// against M5Stack's own Tab5 documentation. Recorded here because the display
// hotfix turned an "unexplained address" into a root cause once, and the next
// person should not have to re-derive the map.
//
//   0x10  ES8388 audio codec
//   0x28  UNIDENTIFIED. Not in any vendor table. Appears and disappears in
//         lockstep with 0x55: absent from the pre-display-fix scan (when
//         LCD_RST was still floating) and present in every scan since
//         LCD_RST began being pulsed. Strongly suggests a second endpoint of
//         the panel/TDDI chip itself rather than a distinct device. Not
//         touched by this firmware.
//   0x32  RX8130CE RTC
//   0x40  ES7210 microphone ADC / AEC front-end
//   0x41  INA226 battery power monitor
//   0x43  PI4IOE5V6408 IO-expander #1 (LCD_RST P4, TP_RST P5, CAM_RST P6,
//         SPK_EN P1, EXT_5V_EN P2)
//   0x44  PI4IOE5V6408 IO-expander #2 (WLAN_PWR_EN P0, USB 5V, charger)
//   0x55  ST7121/ST7123 panel AND its integrated touch engine
//   0x68  BMI270 6-axis IMU
//
// (0x14 / 0x5D would be a GT911 on an ILI9881-revision board; neither is
// present here, which is correct and expected for an ST7121.)
// ---------------------------------------------------------------------------

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
// 16-bit big-endian register addressing used in touch_tab5.cpp.
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

// ---------------------------------------------------------------------------
// HY2.0 peripheral units: NFC, RFID2, RF433R/T (Task 18)
// ---------------------------------------------------------------------------
//
// EXTERNAL I2C bus, for the NFC and RFID2 units (both I2C devices).
// CONFIRMED 2026-08-08 from M5Stack's own product page
// (docs.m5stack.com/en/core/Tab5), whose pin table lists exactly one
// physical HY2.0-4P connector, labelled PORT.A:
//     HY2.0-4P (PORT.A): G53, G54
// This is a DIFFERENT bus from the internal one Tasks 5/6/10 use
// (TAB5_INTERNAL_I2C_SDA_GPIO/SCL_GPIO = 31/32, which carries the display/
// touch/IMU/RTC/codec/IO-expanders and is NOT exposed on any external
// connector). Driven via the second Arduino I2C peripheral, `Wire1`, in
// nfc_pn532.cpp, to keep it electrically and in-code distinct from `Wire`.
// Also cross-confirmed by this project's own variant file
// (boards/variants/quarky_tab5_p4/pins_arduino.h), which already labels
// GPIO 53/54 "Tab5's actual external I2C bus pins" (set there for the
// unrelated reason of keeping Wire.begin()'s no-args fallback off the C6
// SDIO bus, which happens to corroborate the same two pins independently).
#define TAB5_EXTERNAL_I2C_SDA_GPIO 53
#define TAB5_EXTERNAL_I2C_SCL_GPIO 54
// Standard-mode 100 kHz, not the internal bus's 400 kHz: this bus and every
// device on it (PN532 / WS1850S / ST25R3916 -- see below) are, unlike the
// internal bus, entirely unverified on real hardware as of this task.
// Conservative until proven otherwise, matching how the internal touch bus
// itself started out (see the touch hotfix report) before being raised once
// measured. Revisit if verified stable at 400 kHz.
#define TAB5_EXTERNAL_I2C_FREQ_HZ  100000

// EXTERNAL 5V RAIL -- PORT.A's red (5V) wire is NOT powered by default.
//
// Found the same way, and for the same reason, as the C6's WLAN_PWR_EN gate
// above: Task 18's first real-hardware run scanned the whole 0x08-0x77 range
// on GPIO 53/54 with a physical M5Stack NFC unit plugged in and got
// "(nothing responded)" -- not a wrong address, an entirely silent bus. The
// unit had no supply, so it could not ACK anything.
//
// On Tab5 the external 5V bus is gated by EXT_5V_EN, which is NOT a P4 GPIO.
// It is P2 of the *first* PI4IOE5V6408 IO-expander, at address 0x43 (the same
// expander that carries LCD_RST P4 / TP_RST P5 above), on the INTERNAL I2C
// bus (TAB5_INTERNAL_I2C_SDA_GPIO/SCL_GPIO = 31/32) -- i.e. the gate for the
// external bus lives on the internal one.
//
// Three independent sources agree, fetched 2026-08-08:
//   1. M5Stack's own product page, docs.m5stack.com/en/core/Tab5, pinmap
//      section: "EXT_5V_BUS: Provides 5V power to the Tab5 rear M5-Bus, the
//      side 2.54-10P expansion port, and the HY2.0-4P interface", with the
//      output controlled by EXT5V_EN. This is the authoritative statement
//      that PORT.A's 5V pin specifically hangs off this switch.
//   2. espp/m5stack-tab5 BSP (esp-cpp/espp, components/m5stack-tab5/include/
//      m5stack-tab5.hpp):
//        static constexpr int EXT_5V_EN_PIN = (1 << 2); // EXT_5V_EN (via PI4IOE5V6408 P2)
//        IOX_0x43_OUTPUTS = CAM_RST_PIN | TP_RST_PIN | LCD_RST_PIN | EXT_5V_EN_PIN | SPK_EN_PIN;
//        IOX_0x43_DEFAULT_OUTPUTS = IOX_0x43_OUTPUTS; // All outputs high to start
//      i.e. the BSP drives it HIGH as part of generic expander init -- which
//      is exactly why BSP-based projects never notice it is a prerequisite,
//      the identical blind spot that hid WLAN_PWR_EN.
//   3. ESPHome's community Tab5 device config (devices.esphome.io/devices/
//      m5stack-tab5/) declares the 0x43 expander's pin 2 as
//      `external_5v_power` (output), on an i2c bus at sda GPIO31/scl GPIO32.
//
// Active HIGH (all three sources drive/describe it as "enable"). Asserted by
// src/hal/nfc_pn532.cpp before the external bus is scanned.
#define TAB5_EXT_5V_EN_IOEXP_I2C_ADDR 0x43
#define TAB5_EXT_5V_EN_IOEXP_BIT      2    // PI4IOE5V6408 P2 / EXT_5V_EN

// Settle time between asserting EXT_5V_EN and first addressing anything on
// PORT.A. Not a documented figure from any vendor source -- deliberately
// generous, covering the load switch's rise time plus the power-on reset of
// whichever unit is plugged in (M5Stack HY2.0 units regulate 5V down on-board
// and their MCU/reader ICs need their own POR to complete before they will
// ACK). Reduce only with measurements.
#define TAB5_EXT_5V_SETTLE_MS 200

// NFC unit and RFID2 unit I2C addresses.
//
// RFID2: CONFIRMED. docs.m5stack.com/en/product_i2c_addr and
// docs.m5stack.com/en/unit/rfid2 both state Unit RFID2 is a WS1850S chip at
// I2C 0x28. NOTE: this directly corrects the task brief's assumption that
// both the NFC and RFID2 units are PN532-based -- RFID2 is not. (The
// brief's placeholder *address*, 0x28, was nonetheless right; only the
// assumed chip identity was wrong.) See nfc_pn532.cpp's header comment for
// why this does not affect detect()'s correctness -- it is a bare address
// probe, not a PN532-specific protocol exchange.
#define TAB5_RFID2_I2C_ADDR 0x28

// NFC: was UNRESOLVED BY DOCUMENTATION; now RESOLVED ON REAL HARDWARE
// (2026-08-08, HY2.0 port-power hotfix). Two candidates were found in
// research, both real M5Stack/PN532-ecosystem values:
//   0x24 -- PN532 datasheet default I2C address (also an older, likely-EOL
//           M5Stack Grove NFC module referenced in M5Stack community
//           threads as "NFC PN532 grove v1.1"; does not appear in M5Stack's
//           CURRENT product/I2C-address documentation).
//   0x50 -- M5Stack's current "Unit NFC" / "NFC Universal Unit" product,
//           confirmed via docs.m5stack.com/en/unit/Unit_NFC: chip
//           ST25R3916-AQWT, "I2C @0x50 (100K / 400K)".
// Task 18 could not choose between them because its bus scan found nothing at
// all -- the EXT_5V_EN gate above was the reason. With that gate asserted, the
// full 0x08-0x77 sweep of PORT.A returns exactly one device:
//     quarky-tab5:   0x50  ST25R3916 (M5Stack Unit NFC / NFC Universal Unit)
// So the physically-connected unit is the CURRENT ST25R3916-based Unit NFC,
// NOT a PN532. This retires the 0x24 guess. (The `NfcPN532` class name is
// now a misnomer; its detect() is a chip-agnostic address ACK probe, so it is
// still correct -- see nfc_pn532.cpp's header. Renaming is deliberately left
// to whoever implements real read/write, which WILL be chip-specific.)
#define TAB5_NFC_I2C_ADDR 0x50   // CONFIRMED on hardware: ST25R3916 Unit NFC
#define TAB5_NFC_I2C_ADDR_CANDIDATE_PN532_DEFAULT 0x24  // retired; kept for history
#define TAB5_NFC_I2C_ADDR_CANDIDATE_ST25R3916      0x50

// RF433R/T GPIO pins. Both units share Tab5's single HY2.0 PORT.A connector
// with NFC/RFID2 (swapped in one at a time, not simultaneously) -- confirmed
// on real hardware 2026-08-09 by physically swapping a real RFID2 unit for a
// real RF433R unit on the same socket. This corrected the prior research
// conclusion (see rf433_gpio.cpp's older comment) that PORT.A was
// I2C-exclusive.
//
// TAB5_RF433T_PIN: CONFIRMED on real hardware, independently verified. A
// slow (500ms on/500ms off), unmistakable blink was driven on GPIO53 with a
// real RF433T unit attached, while a second physical device (Poseidon
// firmware, tuned to listen at 433MHz) observed real transmitted activity at
// 433.920MHz -- the standard ISM center frequency for this device class --
// precisely correlated with the GPIO53 window and not the (also-tested)
// GPIO54 window. This is real, independent, positive real-hardware evidence,
// not an assumption.
#ifndef TAB5_RF433T_PIN
#define TAB5_RF433T_PIN 53 // CONFIRMED 2026-08-09: independent 433.92MHz listener test
#endif
//
// TAB5_RF433R_PIN: CONFIRMED on real hardware 2026-08-18, to the same
// standard as T above. With a real RF433R unit on PORT.A, an
// interrupt-driven capture (attachInterrupt CHANGE, micros()-timestamped
// edges -- see this plan's Task 1 and src/features/rf433/rf433_common.cpp)
// was run for 20 seconds while a second physical device running Bruce
// firmware transmitted a 433MHz sub continuously. GPIO53 produced a highly
// regular, continuously-sustained burst-repeat pattern -- dozens of evenly
// spaced ~7-8ms burst boundaries across the whole window, each burst made of
// edges spanning a wide but structured pulse-width range. Regular repeating
// structure at a transmitter's cadence is signal; a floating pin produces
// irregular chatter with no such period. This is real, positive
// real-hardware evidence.
//
// It also retires the earlier same-day-2026-08-09 negative result, which
// polled digitalRead(53)/digitalRead(54) from loop() during a remote-control
// button press and saw nothing. That was a false negative from inadequate
// sampling, exactly as suspected at the time: loop()-based polling samples
// far too slowly and irregularly to catch OOK receive-pulse timing (hundreds
// of microseconds per bit). The pin was never wrong -- the instrument was.
#ifndef TAB5_RF433R_PIN
#define TAB5_RF433R_PIN 53 // CONFIRMED 2026-08-18: interrupt-driven receive test, continuous 433MHz TX from a second Bruce-firmware device
#endif

// IR unit GPIO pins (real M5Stack "Unit IR", SKU U002 -- see
// hal/ir_unit.h for the full hardware story and citations). A THIRD real
// consumer of PORT.A's GPIO53/54 pins, alongside external I2C
// (SDA=53/SCL=54) and RF433 (both R/T on 53) -- arbitrated the same way
// via hal/gpio53_arbiter.h's Owner::kIr.
//
// Mapping confirmed directly from two real sources, both 2026-08-21: (1)
// the unit's own datasheet PDF (project owner-supplied,
// ~/Downloads/ir.pdf), whose schematic/pin-map table gives the unit-side
// wire colors Yellow=IR_TX, White=IR_RX; (2) a screenshot of M5Stack's own
// unit-compatibility page (project owner-supplied, same session), which
// maps those same wire colors directly onto Tab5's PORT.A pins: Unit IR's
// PORT.B (GND/5V/IR_TX/IR_RX) <-> Tab5 PORT.A (GND/5V/G53/G54) --
// i.e. IR_TX=G53, IR_RX=G54. Not yet independently re-verified via a
// real GPIO-level bring-up test on this project's own hardware (Task 15's
// own next step) -- this mapping is real-source-cited, not a guess, but
// is still a datasheet claim until that bring-up test confirms it.
#ifndef TAB5_IR_TX_GPIO
#define TAB5_IR_TX_GPIO 53
#endif
#ifndef TAB5_IR_RX_GPIO
#define TAB5_IR_RX_GPIO 54
#endif

// Free GPIOs on the Tab5's rear M5-Bus connector (per docs.m5stack.com/en/
// core/Tab5's pin table, fetched 2026-08-08), recorded here as the most
// likely place a future RF433R/T wiring would land, since PORT.A is already
// spoken for: G16, G17, G18, G45, G19, G52, G7, G6, G3, G4, G2, G48, G47,
// G35, G51, G38, G37, G5 (G31/G32 on that same connector are the internal
// I2C bus, already in use -- exclude those two from consideration).
