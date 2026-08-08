#include "display_tab5.h"

#include <Arduino.h>
#include <Wire.h>
#include <driver/ppa.h>
#include <esp_heap_caps.h>
#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_ldo_regulator.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

#include "../../boards/tab5/pins_config.h"
#include "io_expander.h"
#include "tab5_panel_cmds.h"

// -----------------------------------------------------------------------------
// Real MIPI-DSI panel bring-up for the M5Stack Tab5.
//
// Structure and every hardware constant follow espp's m5stack-tab5 BSP
// (github.com/esp-cpp/espp), specifically
// components/m5stack-tab5/src/video.cpp `M5StackTab5::initialize_lcd()` and
// `M5StackTab5::detect_display_controller()`, fetched 2026-08-07. The two
// per-controller vendor init command tables come from espp's display_drivers
// component and are transcribed in tab5_panel_cmds.h. Pin/timing constants and
// their citations live in boards/tab5/pins_config.h.
//
// BSP sequence, reproduced here in the same order (order matters):
//   1. acquire the MIPI-DSI PHY power LDO channel        (esp_ldo_acquire_channel)
//   2. backlight on
//   3. pulse LCD_RST via the 0x43 I2C IO-expander        (assert/10ms/release/120ms)
//   4. detect the panel controller over I2C              (AFTER reset -- see below)
//   5. esp_lcd_new_dsi_bus       (2 data lanes, per-controller lane bit rate)
//   6. esp_lcd_new_panel_io_dbi  (command channel: 8-bit cmd, 8-bit param)
//   7. esp_lcd_new_panel_dpi     (video stream: NATIVE 720x1280, per-ctrl timing)
//   8. send the controller's vendor init command table over the DBI channel
//   9. esp_lcd_panel_init        (starts the DPI video stream)
//  10. register on_color_trans_done so flush() can wait for the DMA copy
//
// The BSP does NOT call esp_lcd_panel_reset() or esp_lcd_panel_disp_on_off() on
// the DPI panel: the hardware reset is the IO-expander pulse in step 3, and
// "display on" is command 0x29 inside each vendor init table. Neither is added
// here -- esp_lcd's generic DPI panel has no reset/disp_on_off implementation to
// call anyway, and inventing one would deviate from the reference sequence.
// -----------------------------------------------------------------------------

namespace {

esp_ldo_channel_handle_t s_phy_pwr_chan = nullptr;
esp_lcd_dsi_bus_handle_t s_dsi_bus = nullptr;
esp_lcd_panel_io_handle_t s_panel_io = nullptr;
esp_lcd_panel_handle_t s_panel = nullptr;

// Signalled from the DPI driver's ISR when the user draw buffer has been copied
// into the panel's frame buffer, i.e. when the caller may reuse it.
SemaphoreHandle_t s_trans_done = nullptr;
// If the callback never fires (a driver path that copies synchronously and
// doesn't notify), waiting would cost the timeout on every single flush.
// Detect that once and stop waiting for the rest of the boot instead.
bool s_wait_for_trans_done = true;
constexpr TickType_t kTransDoneTimeout = pdMS_TO_TICKS(200);

bool IRAM_ATTR onColorTransDone(esp_lcd_panel_handle_t, esp_lcd_dpi_panel_event_data_t *, void *) {
    BaseType_t higher_woken = pdFALSE;
    if (s_trans_done != nullptr) {
        xSemaphoreGiveFromISR(s_trans_done, &higher_woken);
    }
    return higher_woken == pdTRUE;
}

// Read-only I2C presence check, same shape as touch_gt911.cpp's.
bool i2cProbe(uint8_t addr) {
    Wire.beginTransmission(addr);
    return Wire.endTransmission() == 0;
}

// Push one vendor init table over the DSI command (DBI) channel. This is
// exactly what espp's display drivers do: their `send_commands()` loops the
// table calling `write_command()`, which for Tab5 is
// `M5StackTab5::dsi_write_command()` -> `esp_lcd_panel_io_tx_param()`.
template <typename Cmd>
bool sendInitTable(const Cmd *table, size_t count, const char *name) {
    size_t failures = 0;
    for (size_t i = 0; i < count; i++) {
        const Cmd &c = table[i];
        esp_err_t err = esp_lcd_panel_io_tx_param(s_panel_io, c.cmd,
                                                  c.len ? c.params : nullptr, c.len);
        if (err != ESP_OK) {
            // Log the first few only: a wedged DSI link would otherwise emit
            // hundreds of identical lines and bury the useful diagnostics.
            if (failures < 3) {
                Serial.printf("quarky-tab5: display %s init cmd[%u] 0x%02X failed: %s\n", name,
                              static_cast<unsigned>(i), c.cmd, esp_err_to_name(err));
            }
            failures++;
        }
        if (c.delay_ms) {
            delay(c.delay_ms);
        }
    }
    if (failures) {
        Serial.printf("quarky-tab5: display %s init table: %u/%u commands FAILED\n", name,
                      static_cast<unsigned>(failures), static_cast<unsigned>(count));
    } else {
        Serial.printf("quarky-tab5: display %s init table sent OK (%u commands)\n", name,
                      static_cast<unsigned>(count));
    }
    return failures == 0;
}

// ---------------------------------------------------------------------------
// Rotation: logical landscape -> native portrait.
//
// The panel is natively 720x1280. When TAB5_DISPLAY_ROTATION is 90 or 270 the
// firmware hands LVGL a 1280x720 landscape surface and rotates each flushed
// block here, so nothing above this file has to know. LVGL's own rotation
// support is deliberately not used: lv_display_set_rotation() would still
// require the driver to rotate the pixels (LVGL only rotates the *area*), and
// lv_display_set_matrix_rotation() needs LV_DRAW_TRANSFORM_USE_MATRIX plus a
// full-frame (DIRECT/FULL) render mode, neither of which this project has.
//
// Mapping, for a source block w x h at logical (x1,y1) with LW = logical width
// and LH = logical height:
//   90  (counter-clockwise): native x = ly,          native y = LW-1-lx
//   270 (clockwise):         native x = LH-1-ly,     native y = lx
// Both produce an h x w destination block. Verified against a hand-worked
// 3x2 example and against the PPA's documented rotation direction
// (hal/ppa_types.h: "Picture rotates 90 degrees CCW").
// ---------------------------------------------------------------------------
#if TAB5_DISPLAY_ROTATION != 0

ppa_client_handle_t s_ppa = nullptr;
uint16_t *s_rot_buf = nullptr;
size_t s_rot_buf_bytes = 0;
bool s_ppa_warned = false;

// The PPA's output buffer, when it lives in external (PSRAM) memory, must have
// both its address and its size aligned to the cache line size; the ESP32-P4's
// L2 line is 128 bytes.
constexpr size_t kCacheAlign = 128;

uint16_t *ensureRotBuf(size_t pixels) {
    size_t need = ((pixels * sizeof(uint16_t)) + kCacheAlign - 1) / kCacheAlign * kCacheAlign;
    if (s_rot_buf != nullptr && s_rot_buf_bytes >= need) {
        return s_rot_buf;
    }
    if (s_rot_buf != nullptr) {
        heap_caps_free(s_rot_buf);
        s_rot_buf = nullptr;
        s_rot_buf_bytes = 0;
    }
    s_rot_buf = static_cast<uint16_t *>(
        heap_caps_aligned_alloc(kCacheAlign, need, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (s_rot_buf == nullptr) {
        Serial.printf("quarky-tab5: display rotation buffer alloc FAILED (%u bytes)\n",
                      static_cast<unsigned>(need));
        return nullptr;
    }
    s_rot_buf_bytes = need;
    return s_rot_buf;
}

void rotateSoftware(const uint16_t *src, uint16_t *dst, int w, int h) {
    // dst is h wide and w tall.
    for (int oy = 0; oy < w; oy++) {
        uint16_t *drow = dst + static_cast<size_t>(oy) * h;
#if TAB5_DISPLAY_ROTATION == 90
        const int scol = w - 1 - oy;
        for (int ox = 0; ox < h; ox++) {
            drow[ox] = src[static_cast<size_t>(ox) * w + scol];
        }
#else // 270
        for (int ox = 0; ox < h; ox++) {
            drow[ox] = src[static_cast<size_t>(h - 1 - ox) * w + oy];
        }
#endif
    }
}

bool rotateWithPpa(const uint16_t *src, uint16_t *dst, size_t dst_bytes, int w, int h) {
    if (s_ppa == nullptr) {
        return false;
    }
    ppa_srm_oper_config_t srm = {};
    srm.in.buffer = src;
    srm.in.pic_w = w;
    srm.in.pic_h = h;
    srm.in.block_w = w;
    srm.in.block_h = h;
    srm.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
    srm.out.buffer = dst;
    srm.out.buffer_size = dst_bytes;
    srm.out.pic_w = h; // rotated: width and height swap
    srm.out.pic_h = w;
    srm.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
#if TAB5_DISPLAY_ROTATION == 90
    srm.rotation_angle = PPA_SRM_ROTATION_ANGLE_90;
#else
    srm.rotation_angle = PPA_SRM_ROTATION_ANGLE_270;
#endif
    srm.scale_x = 1.0f;
    srm.scale_y = 1.0f;
    srm.mode = PPA_TRANS_MODE_BLOCKING;
    esp_err_t err = ppa_do_scale_rotate_mirror(s_ppa, &srm);
    if (err != ESP_OK) {
        if (!s_ppa_warned) {
            s_ppa_warned = true;
            Serial.printf("quarky-tab5: PPA rotate failed (%s); using software rotation\n",
                          esp_err_to_name(err));
        }
        // dst now holds partial/stale data -- report failure so the caller
        // redoes the rotation in software rather than flushing garbage.
        return false;
    }
    return true;
}

void initPpa() {
    ppa_client_config_t cfg = {};
    cfg.oper_type = PPA_OPERATION_SRM;
    esp_err_t err = ppa_register_client(&cfg, &s_ppa);
    if (err != ESP_OK) {
        s_ppa = nullptr;
        Serial.printf("quarky-tab5: PPA client register failed (%s); display rotation will "
                      "fall back to software\n",
                      esp_err_to_name(err));
    }
}

#endif // TAB5_DISPLAY_ROTATION != 0

const char *controllerName(DisplayTab5::Controller c) {
    switch (c) {
    case DisplayTab5::Controller::Ili9881:
        return "ILI9881";
    case DisplayTab5::Controller::St7123:
        return "ST7123";
    default:
        return "UNKNOWN";
    }
}

// Runtime hardware-revision detection, transcribed from video.cpp
// `M5StackTab5::detect_display_controller()`. Must run after the LCD reset
// pulse -- see pins_config.h.
//
// One deviation from the BSP, forced by what the physical unit actually does:
// the probes are retried for a short while. The BSP probes once, immediately
// after releasing reset. On the unit this was developed against, 0x55 does not
// ACK that early but IS present ~300 ms later (observed: an unrelated I2C bus
// scan run after panel init lists 0x55, while the pre-init probe here does
// not). A TDDI part's touch engine scans during the display blanking interval,
// so it plausibly only comes up once the DPI video stream is running -- which
// is a chicken-and-egg the retry cannot always win, hence the fallback in
// init() and the post-init confirmation probe.
DisplayTab5::Controller detectController() {
    constexpr int kAttempts = 6;   // ~250 ms total
    constexpr int kSettleMs = 50;
    for (int attempt = 0; attempt < kAttempts; attempt++) {
        const bool gt911 = i2cProbe(TAB5_ILI9881_PROBE_I2C_ADDR);
        const bool st7123 = i2cProbe(TAB5_ST7123_I2C_ADDR);
        if (attempt == 0 || gt911 || st7123) {
            Serial.printf(
                "quarky-tab5: display probe #%d: 0x%02X(GT911/ILI9881)=%s 0x%02X(ST7123)=%s\n",
                attempt, TAB5_ILI9881_PROBE_I2C_ADDR, gt911 ? "ACK" : "-", TAB5_ST7123_I2C_ADDR,
                st7123 ? "ACK" : "-");
        }
        if (gt911) {
            return DisplayTab5::Controller::Ili9881;
        }
        if (st7123) {
            return DisplayTab5::Controller::St7123;
        }
        if (attempt + 1 < kAttempts) {
            delay(kSettleMs);
        }
    }
    return DisplayTab5::Controller::Unknown;
}

} // namespace

void DisplayTab5::init() {
    if (s_panel != nullptr) {
        // Already brought up. The esp_lcd_new_* handles are one-shot hardware
        // resources; re-running init() would fail on a second DSI bus (there is
        // only one) and leak whatever did succeed.
        panel_handle_ = s_panel;
        return;
    }
    Serial.printf("quarky-tab5: display init (panel native %dx%d, logical %dx%d, rotation %d)\n",
                  TAB5_PANEL_NATIVE_WIDTH, TAB5_PANEL_NATIVE_HEIGHT, TAB5_DISP_WIDTH,
                  TAB5_DISP_HEIGHT, TAB5_DISPLAY_ROTATION);

    // --- 1. MIPI-DSI PHY power rail ----------------------------------------
    // The ESP32-P4's DSI PHY is on a separate internal LDO that is off by
    // default; esp_lcd_new_dsi_bus() fails without it.
    if (s_phy_pwr_chan == nullptr) {
        esp_ldo_channel_config_t phy_pwr_cfg = {};
        phy_pwr_cfg.chan_id = TAB5_DSI_PHY_PWR_LDO_CHANNEL;
        phy_pwr_cfg.voltage_mv = TAB5_DSI_PHY_PWR_LDO_VOLTAGE_MV;
        esp_err_t err = esp_ldo_acquire_channel(&phy_pwr_cfg, &s_phy_pwr_chan);
        if (err != ESP_OK) {
            s_phy_pwr_chan = nullptr;
            Serial.printf("quarky-tab5: display FATAL - MIPI-DSI PHY LDO ch%d @%dmV failed: %s\n",
                          TAB5_DSI_PHY_PWR_LDO_CHANNEL, TAB5_DSI_PHY_PWR_LDO_VOLTAGE_MV,
                          esp_err_to_name(err));
            return;
        }
        Serial.println("quarky-tab5: display MIPI-DSI PHY LDO acquired");
    }

    // --- 2. Backlight -------------------------------------------------------
    // The BSP drives this with LEDC PWM for brightness control; a plain HIGH is
    // full brightness, which is all this bring-up needs (and is the behaviour
    // already confirmed working on the physical unit).
    pinMode(TAB5_DISP_BL_GPIO, OUTPUT);
    digitalWrite(TAB5_DISP_BL_GPIO, HIGH);

    // --- 3. Panel hardware reset via the 0x43 IO-expander -------------------
    // display.init() runs before touch.init() in main.cpp, so this is the first
    // user of the internal I2C bus; begin it here. Wire.begin() is idempotent,
    // so touch_gt911.cpp calling it again later is harmless.
    Wire.begin(TAB5_INTERNAL_I2C_SDA_GPIO, TAB5_INTERNAL_I2C_SCL_GPIO);
    bool reset_ok = tab5_ioexp::set_output(TAB5_DISP_RST_IOEXP_I2C_ADDR,
                                           TAB5_DISP_RST_IOEXP_BIT, false);
    delay(10);
    reset_ok = tab5_ioexp::set_output(TAB5_DISP_RST_IOEXP_I2C_ADDR,
                                      TAB5_DISP_RST_IOEXP_BIT, true) &&
               reset_ok;
    delay(120);
    Serial.printf("quarky-tab5: display LCD_RST pulse via 0x%02X P%d: %s\n",
                  TAB5_DISP_RST_IOEXP_I2C_ADDR, TAB5_DISP_RST_IOEXP_BIT,
                  reset_ok ? "OK" : "FAILED");

    // --- 4. Which panel controller is this? ---------------------------------
    controller_ = detectController();
#ifdef TAB5_DISPLAY_FORCE_CONTROLLER
    // Escape hatch. Detection still runs above so the probe results stay in the
    // log (they are the only evidence of which revision this board really is),
    // but the build flag wins.
#if TAB5_DISPLAY_FORCE_CONTROLLER == 1
    controller_ = Controller::Ili9881;
#elif TAB5_DISPLAY_FORCE_CONTROLLER == 2
    controller_ = Controller::St7123;
#else
#error "TAB5_DISPLAY_FORCE_CONTROLLER must be 1 (ILI9881) or 2 (ST7123)"
#endif
    Serial.printf("quarky-tab5: display controller FORCED to %s by build flag\n",
                  controllerName(controller_));
#else
    if (controller_ == Controller::Unknown) {
        // The BSP gives up here. This project cannot: a Tab5 with no picture is
        // unusable, and there is a well-founded fallback. The BSP's ILI9881 test
        // is "does a GT911 answer at 0x14" precisely because the ILI9881
        // revision always ships a standalone GT911 alongside it; no GT911 means
        // it is not that revision, which leaves the ST7123 (whose own touch
        // engine may simply not be answering yet -- see detectController()).
        // Guess loudly rather than silently, and make it overridable.
        Serial.println("quarky-tab5: display controller NOT DETECTED at 0x14 or 0x55 -- "
                       "assuming ST7123 (no GT911 => not the ILI9881 revision). "
                       "Override with -DTAB5_DISPLAY_FORCE_CONTROLLER=1 (ILI9881) or 2 (ST7123).");
        controller_ = Controller::St7123;
    }
#endif
    Serial.printf("quarky-tab5: display controller = %s\n", controllerName(controller_));

    const bool is_ili = (controller_ == Controller::Ili9881);

    // --- 5. MIPI-DSI bus ----------------------------------------------------
    esp_lcd_dsi_bus_config_t bus_config = {};
    bus_config.bus_id = 0;
    bus_config.num_data_lanes = TAB5_DSI_NUM_DATA_LANES;
    bus_config.phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT;
    bus_config.lane_bit_rate_mbps =
        is_ili ? TAB5_DSI_LANE_BIT_RATE_MBPS_ILI9881 : TAB5_DSI_LANE_BIT_RATE_MBPS_ST7123;
    esp_err_t err = esp_lcd_new_dsi_bus(&bus_config, &s_dsi_bus);
    if (err != ESP_OK) {
        Serial.printf("quarky-tab5: display FATAL - esp_lcd_new_dsi_bus failed: %s\n",
                      esp_err_to_name(err));
        return;
    }
    Serial.printf("quarky-tab5: display DSI bus up (%d lanes @ %.0f Mbps)\n",
                  TAB5_DSI_NUM_DATA_LANES, bus_config.lane_bit_rate_mbps);

    // --- 6. DBI command channel --------------------------------------------
    esp_lcd_dbi_io_config_t dbi_config = {};
    dbi_config.virtual_channel = 0;
    dbi_config.lcd_cmd_bits = 8;
    dbi_config.lcd_param_bits = 8;
    err = esp_lcd_new_panel_io_dbi(s_dsi_bus, &dbi_config, &s_panel_io);
    if (err != ESP_OK) {
        Serial.printf("quarky-tab5: display FATAL - esp_lcd_new_panel_io_dbi failed: %s\n",
                      esp_err_to_name(err));
        return;
    }

    // --- 7. DPI video panel (native portrait geometry) ----------------------
    esp_lcd_dpi_panel_config_t dpi_cfg = {};
    dpi_cfg.virtual_channel = 0;
    dpi_cfg.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
    dpi_cfg.pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
    dpi_cfg.num_fbs = 1;
    dpi_cfg.video_timing.h_size = TAB5_PANEL_NATIVE_WIDTH;
    dpi_cfg.video_timing.v_size = TAB5_PANEL_NATIVE_HEIGHT;
    if (is_ili) {
        dpi_cfg.dpi_clock_freq_mhz = TAB5_DPI_CLK_MHZ_ILI9881;
        dpi_cfg.video_timing.hsync_back_porch = TAB5_DPI_HSYNC_BP_ILI9881;
        dpi_cfg.video_timing.hsync_pulse_width = TAB5_DPI_HSYNC_PW_ILI9881;
        dpi_cfg.video_timing.hsync_front_porch = TAB5_DPI_HSYNC_FP_ILI9881;
        dpi_cfg.video_timing.vsync_back_porch = TAB5_DPI_VSYNC_BP_ILI9881;
        dpi_cfg.video_timing.vsync_pulse_width = TAB5_DPI_VSYNC_PW_ILI9881;
        dpi_cfg.video_timing.vsync_front_porch = TAB5_DPI_VSYNC_FP_ILI9881;
    } else {
        dpi_cfg.dpi_clock_freq_mhz = TAB5_DPI_CLK_MHZ_ST7123;
        dpi_cfg.video_timing.hsync_back_porch = TAB5_DPI_HSYNC_BP_ST7123;
        dpi_cfg.video_timing.hsync_pulse_width = TAB5_DPI_HSYNC_PW_ST7123;
        dpi_cfg.video_timing.hsync_front_porch = TAB5_DPI_HSYNC_FP_ST7123;
        dpi_cfg.video_timing.vsync_back_porch = TAB5_DPI_VSYNC_BP_ST7123;
        dpi_cfg.video_timing.vsync_pulse_width = TAB5_DPI_VSYNC_PW_ST7123;
        dpi_cfg.video_timing.vsync_front_porch = TAB5_DPI_VSYNC_FP_ST7123;
    }
    dpi_cfg.flags.use_dma2d = true;
    err = esp_lcd_new_panel_dpi(s_dsi_bus, &dpi_cfg, &s_panel);
    if (err != ESP_OK) {
        s_panel = nullptr;
        Serial.printf("quarky-tab5: display FATAL - esp_lcd_new_panel_dpi failed: %s\n",
                      esp_err_to_name(err));
        return;
    }
    Serial.printf("quarky-tab5: display DPI panel created (%dx%d @ %.0f MHz)\n",
                  TAB5_PANEL_NATIVE_WIDTH, TAB5_PANEL_NATIVE_HEIGHT, dpi_cfg.dpi_clock_freq_mhz);

    // --- 8. Vendor init command table ---------------------------------------
    if (is_ili) {
        sendInitTable(kTab5Ili9881Init,
                      sizeof(kTab5Ili9881Init) / sizeof(kTab5Ili9881Init[0]), "ILI9881");
    } else {
        sendInitTable(kTab5St7123Init, sizeof(kTab5St7123Init) / sizeof(kTab5St7123Init[0]),
                      "ST7123");
    }

    // --- 9. Start the video stream ------------------------------------------
    err = esp_lcd_panel_init(s_panel);
    if (err != ESP_OK) {
        Serial.printf("quarky-tab5: display FATAL - esp_lcd_panel_init failed: %s\n",
                      esp_err_to_name(err));
        s_panel = nullptr;
        return;
    }

    // --- 10. Flush-completion plumbing --------------------------------------
    s_trans_done = xSemaphoreCreateBinary();
    if (s_trans_done == nullptr) {
        // Without it flush() cannot know when the draw buffer is free again;
        // stop waiting rather than blocking forever (worst case is tearing).
        s_wait_for_trans_done = false;
        Serial.println("quarky-tab5: display flush semaphore alloc failed (tearing possible)");
    } else {
        esp_lcd_dpi_panel_event_callbacks_t cbs = {};
        cbs.on_color_trans_done = onColorTransDone;
        err = esp_lcd_dpi_panel_register_event_callbacks(s_panel, &cbs, nullptr);
        if (err != ESP_OK) {
            s_wait_for_trans_done = false;
            Serial.printf("quarky-tab5: display trans-done callback register failed: %s "
                          "(tearing possible)\n",
                          esp_err_to_name(err));
        }
    }

#if TAB5_DISPLAY_ROTATION != 0
    initPpa();
#endif

    panel_handle_ = s_panel;

    // Post-init confirmation. If detection had to fall back to an assumption
    // above, this is where it gets checked against reality: a TDDI ST7123's
    // I2C endpoint shows up once the video stream is running, so an ACK at 0x55
    // here turns "assumed ST7123" into "confirmed ST7123" in the boot log.
    if (controller_ == Controller::St7123) {
        Serial.printf("quarky-tab5: display post-init probe 0x%02X(ST7123)=%s\n",
                      TAB5_ST7123_I2C_ADDR,
                      i2cProbe(TAB5_ST7123_I2C_ADDR) ? "ACK (controller confirmed)"
                                                     : "- (still silent; if the screen is blank, "
                                                       "try -DTAB5_DISPLAY_FORCE_CONTROLLER=1)");
    }

    Serial.printf("quarky-tab5: display READY (%s, %dx%d logical)\n", controllerName(controller_),
                  TAB5_DISP_WIDTH, TAB5_DISP_HEIGHT);
}

void DisplayTab5::flush(int x1, int y1, int x2, int y2, const uint16_t *colors) {
    if (s_panel == nullptr || colors == nullptr) {
        return; // panel bring-up failed; UI still runs, just invisible
    }
    if (x1 < 0 || y1 < 0 || x2 < x1 || y2 < y1) {
        return;
    }

    const int w = x2 - x1 + 1;
    const int h = y2 - y1 + 1;

#if TAB5_DISPLAY_ROTATION == 0
    (void)w;
    (void)h;
    const int nx1 = x1;
    const int ny1 = y1;
    const int nx2 = x2 + 1; // esp_lcd's end coordinates are exclusive
    const int ny2 = y2 + 1;
    const void *src = colors;
#else
    uint16_t *rot = ensureRotBuf(static_cast<size_t>(w) * h);
    if (rot == nullptr) {
        return;
    }
    if (!rotateWithPpa(colors, rot, s_rot_buf_bytes, w, h)) {
        rotateSoftware(colors, rot, w, h);
    }
    // The rotated block is h wide and w tall; place it in native coordinates.
#if TAB5_DISPLAY_ROTATION == 90
    const int nx1 = y1;
    const int ny1 = TAB5_DISP_WIDTH - 1 - x2;
#else // 270
    const int nx1 = TAB5_DISP_HEIGHT - 1 - y2;
    const int ny1 = x1;
#endif
    const int nx2 = nx1 + h;
    const int ny2 = ny1 + w;
    const void *src = rot;
#endif

    if (s_trans_done != nullptr) {
        // Drop any stale completion left over from a previous timed-out flush
        // so the wait below cannot be satisfied by the wrong transfer.
        xSemaphoreTake(s_trans_done, 0);
    }
    // One-shot proof-of-life. "No errors in the log" does not distinguish
    // "flushing fine" from "flush() was never called at all", and this driver
    // is otherwise only observable by looking at the screen.
    static bool logged_first_flush = false;
    if (!logged_first_flush) {
        logged_first_flush = true;
        Serial.printf("quarky-tab5: display first flush: logical (%d,%d)-(%d,%d) -> "
                      "native (%d,%d)-(%d,%d)\n",
                      x1, y1, x2, y2, nx1, ny1, nx2 - 1, ny2 - 1);
    }

    esp_err_t err = esp_lcd_panel_draw_bitmap(s_panel, nx1, ny1, nx2, ny2, src);
    if (err != ESP_OK) {
        static bool warned = false;
        if (!warned) {
            warned = true;
            Serial.printf("quarky-tab5: display draw_bitmap(%d,%d,%d,%d) failed: %s\n", nx1, ny1,
                          nx2, ny2, esp_err_to_name(err));
        }
        return;
    }

    // lvgl_port.cpp calls lv_display_flush_ready() as soon as this returns, so
    // this call must not return until the driver has finished reading `src`.
    // For a DPI panel the copy into the frame buffer is asynchronous (DMA2D);
    // on_color_trans_done is documented as "the draw buffer can be recycled
    // safely". Without this wait LVGL would start re-rendering into a buffer
    // still being read.
    if (s_wait_for_trans_done && s_trans_done != nullptr) {
        if (xSemaphoreTake(s_trans_done, kTransDoneTimeout) != pdTRUE) {
            s_wait_for_trans_done = false;
            Serial.println("quarky-tab5: display flush completion never signalled -- no longer "
                           "waiting (tearing possible)");
        }
    }
}
