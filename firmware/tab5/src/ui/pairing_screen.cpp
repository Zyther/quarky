#include "pairing_screen.h"
#include "screen_scaffold.h"
#include "screen_stack.h"
#include "../hal/psk_store.h"
#include "../hal/c2link_wifi.h"
#include <crypto.h>
#include <qrcode.h>
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <cstdio>

// Real global instance + free function, both defined in main.cpp (matching
// this project's established `extern StorageSD storage;` idiom used by
// every other IStorage-consuming feature module -- e.g. ir_clone.cpp,
// ir_learn.cpp). Needed here as of the 2026-08-23 lazy-WiFi-C2 change (see
// hal/c2link_wifi.h's own header comment): this screen is now the one real
// place C2LinkWifi::init() is called from, instead of main.cpp's boot
// sequence.
extern C2LinkWifi c2link_wifi;
extern void on_c2_receive(const c2proto::Frame &frame);

// Same literal SSID/password/port main.cpp's boot sequence used to pass to
// c2link_wifi.init() before the lazy-init change -- moved here since this
// is now the only real caller.
static const char *const kWifiApSsid = "Quarky-Tab5-Test";
static const char *const kWifiApPassword = "quarkytest123";
static constexpr uint16_t kWifiPort = 7777;

static constexpr int32_t kCanvasSize = 300;

// A static 300x300 RGB565 buffer (176KB) doesn't fit internal SRAM alongside
// the rest of this build's static footprint (LVGL, esp-hosted, BLE stack,
// etc.) -- confirmed empirically: linking it in overflowed internal RAM by
// ~290KB (`ld: --enable-non-contiguous-regions discards section ... Total
// discarded sections size is 289991 bytes`). Follow lvgl_port.cpp's
// established pattern instead: allocate from PSRAM (available per
// platformio.ini's -DBOARD_HAS_PSRAM) with heap_caps_malloc.
//
// This buffer is allocate-once, reuse-forever (module-level static, lazily
// allocated on first use): build_pairing_screen()/render_qr_canvas() run
// every time the user taps "Pair Satellite", and LVGL's raw-pointer canvas
// API never takes ownership of a caller-supplied buffer -- lv_obj_delete()
// on the canvas widget (via ScreenStack::pop()) frees the widget, not this
// buffer. Allocating fresh on every open, as an earlier version of this code
// did, leaked ~180KB of PSRAM per visit to this screen with nothing ever
// freeing it.
static lv_color_t *s_canvas_buf = nullptr;

// Renders the PSK as both a scannable QR code and a 32-character hex string
// underneath it (Cardputer-ADV has no camera, so the hex string is the
// primary pairing path there; the QR code is for any future/companion
// device that does have a camera). The hex label is created first and does
// not depend on the canvas buffer at all, so a PSRAM-exhaustion failure
// below only drops the QR code, not the primary (hex) pairing mechanism.
static void render_qr_canvas(lv_obj_t *parent, const uint8_t psk[16]) {
    char hex[33];
    for (int i = 0; i < 16; i++) sprintf(hex + i * 2, "%02X", psk[i]);
    hex[32] = '\0';

    lv_obj_t *hex_label = lv_label_create(parent);
    lv_label_set_text(hex_label, hex); // shown alongside the QR since Cardputer-ADV has no camera
    // No lv_obj_align() here: `parent` is the scaffold's flex-managed content
    // area, which owns child placement. Mixing manual alignment with a flex
    // parent is what let this screen's widgets overlap each other before.

    QRCode qr;
    uint8_t qr_data[qrcode_getBufferSize(4)];
    qrcode_initText(&qr, qr_data, 4, ECC_MEDIUM, hex);

    if (s_canvas_buf == nullptr) {
        size_t canvas_buf_size = static_cast<size_t>(kCanvasSize) * kCanvasSize * sizeof(lv_color_t);
        s_canvas_buf = static_cast<lv_color_t *>(heap_caps_malloc(canvas_buf_size, MALLOC_CAP_SPIRAM));
        if (s_canvas_buf == nullptr) {
            Serial.printf(
                "quarky-tab5: FATAL - failed to allocate %u bytes for pairing QR canvas (PSRAM)\n",
                static_cast<unsigned>(canvas_buf_size));
            return;
        }
    }

    lv_obj_t *canvas = lv_canvas_create(parent);
    lv_canvas_set_buffer(canvas, s_canvas_buf, kCanvasSize, kCanvasSize, LV_COLOR_FORMAT_RGB565);
    lv_canvas_fill_bg(canvas, lv_color_white(), LV_OPA_COVER);

    int scale = kCanvasSize / qr.size;
    for (int y = 0; y < qr.size; y++) {
        for (int x = 0; x < qr.size; x++) {
            if (qrcode_getModule(&qr, x, y)) {
                for (int dy = 0; dy < scale; dy++)
                    for (int dx = 0; dx < scale; dx++)
                        lv_canvas_set_px(canvas, x * scale + dx, y * scale + dy, lv_color_black(), LV_OPA_COVER);
            }
        }
    }
}

lv_obj_t *build_pairing_screen() {
    // Menu-bar Back button + flex content area, same as every other
    // sub-screen. This screen previously hand-aligned its Back button to
    // TOP_LEFT (10,10) over hand-aligned content, the same pattern that put
    // the keyboard test screen's Back button underneath its text area -- see
    // ui/screen_scaffold.cpp.
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("Pair Satellite", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(content, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    uint8_t psk[16];
    if (!PskStore::load(psk)) {
        c2proto::generate_psk(psk);
        PskStore::save(psk);
        Serial.println("quarky-tab5: generated and persisted new PSK");
    } else {
        Serial.println("quarky-tab5: loaded existing PSK from NVS");
    }
    // Task 20 verification aid: the hex string below is the same one
    // rendered on-screen (see render_qr_canvas) for a human to read off the
    // display and hardcode into Cardputer-ADV's test_psk. Logging it here too
    // means it's also readable over the physically-attached USB serial
    // connection -- useful for bench/CI verification without needing someone
    // to look at the screen. Not a networked exposure; this is a local
    // pairing flow already designed to be human-read off-device.
    {
        char psk_hex[33];
        for (int i = 0; i < 16; i++) sprintf(psk_hex + i * 2, "%02X", psk[i]);
        psk_hex[32] = '\0';
        Serial.printf("quarky-tab5: pairing PSK (hex) = %s\n", psk_hex);
    }

    render_qr_canvas(content, psk);

    // WiFi C2 opt-in (2026-08-23, real hardware finding -- see this file's
    // top-of-file comment and hal/c2link_wifi.h's own header for the full
    // citation): WiFi C2 is no longer brought up unconditionally at boot
    // because doing so cost ~146KB of this board's real ~187KB DMA-capable
    // memory pool regardless of whether anything used it, starving SD reads
    // for other Tab5-native features. BLE C2 is the default; this button is
    // the one real way left to opt into WiFi C2's higher-throughput
    // transport, paying its real memory cost only when actually wanted.
    lv_obj_t *wifi_label = lv_label_create(content);
    lv_label_set_long_mode(wifi_label, LV_LABEL_LONG_WRAP);
    bool already_up = c2link_wifi.is_initialized();
    lv_label_set_text(wifi_label, already_up
                                       ? "WiFi C2: enabled"
                                       : "WiFi C2: off by default (BLE C2 handles "
                                         "pairing/control) -- costs real DMA "
                                         "memory other SD-heavy features need. "
                                         "Enable only if you need the WiFi "
                                         "transport specifically.");

    if (!already_up) {
        lv_obj_t *wifi_btn = lv_button_create(content);
        lv_obj_t *wifi_btn_label = lv_label_create(wifi_btn);
        lv_label_set_text(wifi_btn_label, "Enable WiFi Link");
        lv_obj_add_event_cb(wifi_btn, [](lv_event_t *e) {
            lv_obj_t *btn = static_cast<lv_obj_t *>(lv_event_get_target(e));
            lv_obj_t *label = static_cast<lv_obj_t *>(lv_event_get_user_data(e));
            // Re-load rather than capture the outer psk[16] -- that stack
            // array is long out of scope by the time a real button tap
            // fires this callback. PskStore::load() is a cheap NVS read,
            // and both call sites (this one, and the one that already ran
            // moments ago to render the QR/hex above) always agree on the
            // same persisted key -- see this file's own Task 20 comment.
            uint8_t reload_psk[16];
            if (!PskStore::load(reload_psk)) {
                // Should not happen -- build_pairing_screen() already
                // generated+persisted one above if none existed -- but
                // refuse rather than init() with an undefined key if it
                // somehow does.
                lv_label_set_text(label, "WiFi C2: failed (no PSK found)");
                return;
            }
            bool ok = c2link_wifi.init(reload_psk, kWifiApSsid, kWifiApPassword, kWifiPort);
            if (ok) {
                c2link_wifi.set_receive_handler(on_c2_receive);
                lv_label_set_text(label, "WiFi C2: enabled");
                lv_obj_add_state(btn, LV_STATE_DISABLED);
            } else {
                lv_label_set_text(label, "WiFi C2: failed to start (see serial log)");
            }
        }, LV_EVENT_CLICKED, wifi_label);
    }

    return screen;
}
