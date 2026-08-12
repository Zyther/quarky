#include "lvgl_port.h"
#include <lvgl.h>
#include <src/draw/lv_draw_buf_private.h>
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

// -----------------------------------------------------------------------------
// Draw-buffer allocator: PSRAM, not LVGL's builtin pool.
//
// THIS PREVENTS A HARD DEVICE LOCK-UP, it is not a memory-tuning nicety.
// Measured on real hardware 2026-08-12 while debugging the WiFi Scan freeze
// (see features/wifi/wifi_scan.cpp and the task-3 hang report):
//
//   [Warn] lv_draw_buf_create_ex: No memory: 1240x10, cf: 16, stride: 4960,
//          49600Byte
//   [Warn] lv_draw_layer_alloc_buf: Allocating layer buffer failed. Try later
//
// LVGL renders any object that needs its own layer (anything with opacity,
// a blend mode, a transform, or a bitmap mask) into a temporary layer buffer
// first. For a full-width row strip on this 1280x720 panel that buffer is
// ~49.6 kB of ARGB8888 -- most of LV_MEM_SIZE (64 kB), which by default is
// ALSO where every widget, style and label string lives. With a screenful of
// widgets already resident the allocation simply cannot succeed.
//
// And an allocation failure here is not a degraded render, it is fatal:
// lv_draw_buf_create() returns NULL -> lv_draw_layer_alloc_buf() returns NULL
// -> the software draw unit reports LV_DRAW_UNIT_IDLE -> and because
// LV_USE_OS is LV_OS_NONE, lv_refr.c's draw_buf_flush() sits in
//
//     while(layer->draw_task_head) { lv_draw_dispatch_wait_for_request();
//                                    lv_draw_dispatch(); }
//
// with lv_draw_dispatch_wait_for_request() compiled down to a bare
// `while(!dispatch_req);`. Nothing ever frees memory from inside that loop, so
// it never terminates: lv_timer_handler() never returns, and the Arduino loop
// task spins at priority 1 on core 1 forever. Neither watchdog catches it --
// the task WDT only monitors IDLE0 (CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1
// is not set in this framework's sdkconfig) and the interrupt WDT is happy
// because interrupts are still being serviced. The result is a totally silent
// brick: frozen screen, dead touch, no serial, no panic, no reboot, recoverable
// only by pulling power. That is exactly the WiFi Scan bug.
//
// Routing draw buffers to PSRAM fixes it at the source. The board has ~30 MB
// of SPIRAM free versus 64 kB of LVGL pool, and these buffers are transient,
// CPU-only (the software renderer writes them, then blends them into the main
// draw buffer -- no DMA touches them, so no cache maintenance is needed
// beyond what the display driver already does for its own buffer). LVGL's
// builtin pool is left to widgets/styles alone, which it comfortably fits:
// the whole 21-row scan list measured 36% of it.
static void *psram_draw_buf_malloc(size_t size, lv_color_format_t) {
    // LVGL's own default over-allocates by LV_DRAW_BUF_ALIGN - 1 so that
    // align_pointer_cb can round the returned pointer up inside the block;
    // keep that contract, since we are only replacing malloc/free and the
    // default align/stride callbacks still apply.
    size += LV_DRAW_BUF_ALIGN - 1;
    void *p = heap_caps_malloc(size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (p == nullptr) {
        // Small buffers are still worth trying in internal RAM -- falling back
        // keeps a PSRAM hiccup from re-arming the freeze described above.
        p = heap_caps_malloc(size, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }
    return p;
}

static void psram_draw_buf_free(void *buf) {
    heap_caps_free(buf);
}

// LVGL's warnings are the only reason the freeze above was diagnosable at all;
// with LV_USE_LOG 0 (the previous setting) LVGL failed completely silently.
// Kept on permanently at WARN level -- it is quiet in normal operation and
// turns the next LVGL resource failure into a log line instead of a mystery.
static void lvgl_log_cb(lv_log_level_t level, const char *buf) {
    Serial.printf("quarky-tab5: [lvgl:%d] %s\n", (int)level, buf);
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

    lv_log_register_print_cb(lvgl_log_cb);

    // Must come after lv_init() (which installs the defaults) and before any
    // widget is created. Both the general and the image handler sets are
    // redirected: layers come from the general set, and decoded/rescaled image
    // buffers from the image set are the other allocation big enough to
    // exhaust the builtin pool. Font glyph buffers are deliberately left on
    // the default -- they are small, extremely hot, and better off in
    // internal RAM.
    lv_draw_buf_handlers_t *draw_handlers = lv_draw_buf_get_handlers();
    draw_handlers->buf_malloc_cb = psram_draw_buf_malloc;
    draw_handlers->buf_free_cb = psram_draw_buf_free;
    lv_draw_buf_handlers_t *image_handlers = lv_draw_buf_get_image_handlers();
    image_handlers->buf_malloc_cb = psram_draw_buf_malloc;
    image_handlers->buf_free_cb = psram_draw_buf_free;

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
    // Cache-line aligned (ESP32-P4 L2 line = 128 bytes), and the size rounded up
    // to a whole number of lines. DisplayTab5::flush() hands this buffer to the
    // PPA (hardware rotation) and to the DSI DPI panel's DMA2D copy, both of
    // which move data behind the CPU's cache; an unaligned PSRAM buffer makes
    // the driver-side cache sync fail (ESP_ERR_INVALID_ARG) or, worse, share a
    // cache line with an unrelated allocation. heap_caps_aligned_alloc requires
    // the size to be a multiple of the alignment too.
    constexpr size_t kCacheAlign = 128;
    buf_size = (buf_size + kCacheAlign - 1) / kCacheAlign * kCacheAlign;
    s_draw_buf = static_cast<uint16_t *>(
        heap_caps_aligned_alloc(kCacheAlign, buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (s_draw_buf == nullptr) {
        // Without this check, a failed allocation feeds a null buffer into
        // lv_display_set_buffers() and the first draw hits lv_conf.h's
        // LV_USE_ASSERT_NULL (1), whose LV_ASSERT_HANDLER is a bare
        // `while(1);`. LV_USE_LOG is enabled now (see include/lv_conf.h --
        // flipped on 2026-08-12 during the WiFi Scan freeze investigation),
        // so that path would at least log something today, but this explicit
        // check stays: it names the exact failing size, and still saves
        // whoever's debugging a hung boot from a silent while(1) if
        // LV_USE_LOG is ever turned back off. Log the requested size and
        // halt predictably.
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
