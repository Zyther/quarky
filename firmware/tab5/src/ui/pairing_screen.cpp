#include "pairing_screen.h"
#include "screen_stack.h"
#include "../hal/psk_store.h"
#include <crypto.h>
#include <qrcode.h>
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <cstdio>

// Renders the PSK as both a scannable QR code and a 32-character hex string
// underneath it (Cardputer-ADV has no camera, so the hex string is the
// primary pairing path there; the QR code is for any future/companion
// device that does have a camera).
static void render_qr_canvas(lv_obj_t *parent, const uint8_t psk[16]) {
    char hex[33];
    for (int i = 0; i < 16; i++) sprintf(hex + i * 2, "%02X", psk[i]);
    hex[32] = '\0';

    QRCode qr;
    uint8_t qr_data[qrcode_getBufferSize(4)];
    qrcode_initText(&qr, qr_data, 4, ECC_MEDIUM, hex);

    // A static 300x300 RGB565 buffer (176KB) doesn't fit internal SRAM
    // alongside the rest of this build's static footprint (LVGL, esp-hosted,
    // BLE stack, etc.) -- confirmed empirically: linking it in overflowed
    // internal RAM by ~290KB (`ld: --enable-non-contiguous-regions discards
    // section ... Total discarded sections size is 289991 bytes`). Follow
    // lvgl_port.cpp's established pattern instead: allocate from PSRAM
    // (available per platformio.ini's -DBOARD_HAS_PSRAM) with
    // heap_caps_malloc, since this buffer only needs to exist transiently
    // while the pairing screen is open.
    constexpr int32_t kCanvasSize = 300;
    size_t canvas_buf_size = static_cast<size_t>(kCanvasSize) * kCanvasSize * sizeof(lv_color_t);
    auto *canvas_buf = static_cast<lv_color_t *>(heap_caps_malloc(canvas_buf_size, MALLOC_CAP_SPIRAM));
    if (canvas_buf == nullptr) {
        Serial.printf(
            "quarky-tab5: FATAL - failed to allocate %u bytes for pairing QR canvas (PSRAM)\n",
            static_cast<unsigned>(canvas_buf_size));
        return;
    }

    lv_obj_t *canvas = lv_canvas_create(parent);
    lv_canvas_set_buffer(canvas, canvas_buf, kCanvasSize, kCanvasSize, LV_COLOR_FORMAT_RGB565);
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

    lv_obj_t *hex_label = lv_label_create(parent);
    lv_label_set_text(hex_label, hex); // shown alongside the QR since Cardputer-ADV has no camera
    lv_obj_align(hex_label, LV_ALIGN_BOTTOM_MID, 0, -10);
}

lv_obj_t *build_pairing_screen() {
    lv_obj_t *screen = lv_obj_create(nullptr);

    uint8_t psk[16];
    if (!PskStore::load(psk)) {
        c2proto::generate_psk(psk);
        PskStore::save(psk);
        Serial.println("quarky-tab5: generated and persisted new PSK");
    } else {
        Serial.println("quarky-tab5: loaded existing PSK from NVS");
    }

    render_qr_canvas(screen, psk);

    lv_obj_t *back = lv_button_create(screen);
    lv_obj_align(back, LV_ALIGN_TOP_LEFT, 10, 10);
    lv_obj_t *back_label = lv_label_create(back);
    lv_label_set_text(back_label, "Back");
    lv_obj_add_event_cb(back, [](lv_event_t *e) { ScreenStack::pop(); }, LV_EVENT_CLICKED, nullptr);

    return screen;
}
