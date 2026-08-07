#include "lvgl_port.h"
#include <lvgl.h>
#include <Arduino.h>
#include <esp_heap_caps.h>

static IDisplay *s_display = nullptr;
static ITouch *s_touch = nullptr;
static lv_display_t *s_lv_display = nullptr;

// LVGL 9's lv_display_create() does NOT allocate a draw buffer -- the brief's
// reference lvgl_port.cpp omits this, but lv_display_set_buffers() is
// mandatory or lv_timer_handler() has nothing to render into (confirmed via
// lvgl/src/display/lv_display.h at the v9.2.0 tag). Use a partial buffer
// covering a strip of rows rather than the full 1280x720 frame (1.8MB at
// RGB565) to keep this to a modest allocation; PSRAM is available per
// platformio.ini's -DBOARD_HAS_PSRAM.
static constexpr int kDrawBufLines = 40;
static uint16_t *s_draw_buf = nullptr;

static void flush_cb(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
    s_display->flush(area->x1, area->y1, area->x2, area->y2, (const uint16_t *)px_map);
    lv_display_flush_ready(disp);
}

static void touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data) {
    int16_t x, y;
    bool pressed;
    s_touch->read(x, y, pressed);
    data->point.x = x;
    data->point.y = y;
    data->state = pressed ? LV_INDEV_STATE_PRESSED : LV_INDEV_STATE_RELEASED;
}

void lvgl_port_init(IDisplay &display, ITouch &touch) {
    s_display = &display;
    s_touch = &touch;

    lv_init();

    // LVGL 9 dropped the compile-time LV_TICK_CUSTOM macro (see
    // include/lv_conf.h's header comment) in favor of a runtime callback.
    // Arduino's millis() (uint32_t, ms since boot) matches lv_tick_get_cb_t
    // exactly, so hand it straight to LVGL as the tick source. Without this,
    // lv_timer_handler()/animations/indev click-debounce never advance since
    // nothing else calls lv_tick_inc().
    // Source: lvgl/src/tick/lv_tick.h (v9.2.0 tag), `lv_tick_set_cb()`.
    lv_tick_set_cb(millis);

    s_lv_display = lv_display_create(display.width(), display.height());
    lv_display_set_flush_cb(s_lv_display, flush_cb);

    size_t buf_size = static_cast<size_t>(display.width()) * kDrawBufLines * sizeof(uint16_t);
    s_draw_buf = static_cast<uint16_t *>(heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM));
    if (s_draw_buf == nullptr) {
        // Without this check, a failed allocation feeds a null buffer into
        // lv_display_set_buffers() and the first draw hits lv_conf.h's
        // LV_USE_ASSERT_NULL (1). With LV_USE_LOG left at 0 (its shipped
        // default -- see include/lv_conf.h), LV_ASSERT_HANDLER's `while(1);`
        // fires with zero serial output: a silent hang right before real
        // hardware bring-up starts, with nothing for whoever's debugging a
        // hung boot to go on. Log the requested size and halt predictably
        // instead.
        Serial.printf(
            "quarky-tab5: FATAL - failed to allocate %u bytes for LVGL draw buffer (PSRAM)\n",
            static_cast<unsigned>(buf_size));
        while (true) {
            delay(1000);
        }
    }
    lv_display_set_buffers(s_lv_display, s_draw_buf, nullptr, buf_size,
                            LV_DISPLAY_RENDER_MODE_PARTIAL);

    lv_indev_t *indev = lv_indev_create();
    lv_indev_set_type(indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(indev, touch_read_cb);
}

void lvgl_port_tick() {
    lv_timer_handler();
}
