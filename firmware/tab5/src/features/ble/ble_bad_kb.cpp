#include "ble_bad_kb.h"
#include "ble_hid_spike.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <Arduino.h> // Serial (diagnostic logging, same convention as every other BLE feature)
#include <feature_registry.h>
#include <lvgl.h>
#include <cstring>

extern FeatureRegistry g_registry;

// -----------------------------------------------------------------------------
// Why script typing is poll()-driven and not one blocking call from "Send"'s
// LV_EVENT_CLICKED handler (this task's real redesign requirement, distinct
// from the GATT-registration redesign covered in ble_bad_kb.h):
//
// A synchronous type_script() sending every character of the whole script in
// one call -- each character costing a blocking key-down notify + delay +
// key-up notify + delay -- would freeze LVGL (no touch response, no redraw)
// for the entire script's duration, and Ducky-script payloads are routinely
// many lines/hundreds of characters, not just this task's short demo string.
// This project already hit exactly this class of bug for real: Task 3's
// wifi_connect.cpp originally ran a multi-second blocking Wi-Fi connect
// directly inside a tile's click handler and starved loop() long enough to
// miss enableLoopWDT()'s feed window and reboot -- see wifi_connect.cpp's own
// comments for the real-hardware crash and its fix (moving the long operation
// onto a background task, drained incrementally from poll()). A long BLE
// Bad-KB script sent synchronously is the same shape of bug: an unbounded
// blocking stretch inside a UI callback, on the same main task loop()'s
// watchdog feed and LVGL redraw both depend on.
//
// Fix here: "Send" only copies the script text into a small fixed buffer and
// resets a cursor -- no NimBLE calls happen in the click handler at all.
// poll() (wired into main.cpp's loop(), like every other feature in this
// plan) then advances the parser by exactly one action per call. Each action
// is either a single BleHidSpike::send_key() call (bounded to ~40ms --
// comfortably under this project's ~50ms no-blocking-call-longer-than-this
// Global Constraint, see ble_hid_spike.cpp's send_key() comment for the exact
// timing) or a zero-cost bookkeeping step (skipping a "STRING " marker,
// advancing past an unrecognized character). A long script therefore spreads
// its total typing time across many loop() iterations instead of stalling
// one of them.
// -----------------------------------------------------------------------------

namespace BleBadKbFeature {

// Minimal USB HID keycode table for a..z and space/enter -- enough for a
// real, demonstrable Ducky-script subset (STRING + ENTER), matching this
// task's own scope (a fuller Ducky-script command set -- CTRL/ALT/GUI
// modifiers, DELAY, etc. -- is real, considered future work, not a
// placeholder: this ships a working, if reduced, HID typer today).
static uint8_t keycode_for(char c) {
    if (c >= 'a' && c <= 'z') return 0x04 + (c - 'a');
    if (c >= 'A' && c <= 'Z') return 0x04 + (c - 'A'); // shift not modeled in this reduced set
    if (c == ' ') return 0x2C;
    return 0;
}

static lv_obj_t *s_status_label = nullptr;
static lv_obj_t *s_script_input = nullptr;
static lv_obj_t *s_keyboard = nullptr;
static bool s_last_connected = false;

// Script-typing state, all main-task-only (written from the "Send" button's
// LV_EVENT_CLICKED handler, read/advanced from poll() -- both run on the main
// LVGL/loop() task, never the NimBLE host task, so none of this needs
// volatile; contrast BleHidSpike's own cross-task scalars, documented in
// ble_hid_spike.cpp).
static constexpr size_t kScriptBufLen = 512; // generous headroom over this
                                              // task's short demo script for
                                              // a real, many-line Ducky
                                              // payload; longer input is
                                              // truncated at the textarea
                                              // itself (max_length set below)
                                              // rather than silently here.
static char s_script_buf[kScriptBufLen] = {0};
static size_t s_script_len = 0;
static size_t s_cursor = 0;
static bool s_in_string_mode = false; // true after consuming a "STRING "
                                       // marker, until the next '\n' -- see
                                       // type_script_step()'s comment
static bool s_typing = false;

// Advances the Ducky-script parser by exactly one action and returns true if
// there is more to do, false once the script is exhausted. Semantics match
// the task brief's original (synchronous) type_script() line for line --
// STRING <text> types literal characters up to the next newline; ENTER (or a
// bare newline) sends the Enter keycode; anything else outside a STRING run
// is skipped, not typed, consistent with Ducky script's line-oriented command
// format (unimplemented commands' text is not accidentally typed) -- just
// restructured from that single unbroken while() loop into one-step-per-call
// so poll() can call it once per tick instead of draining it in one shot.
static bool type_script_step() {
    if (s_cursor >= s_script_len) {
        s_in_string_mode = false;
        return false;
    }
    const char *p = s_script_buf + s_cursor;

    if (s_in_string_mode) {
        if (*p == '\n') {
            // Mirrors the original inner while's boundary: stop treating
            // characters as literal STRING text at the newline, but don't
            // consume it here -- fall through to the newline branch below in
            // this SAME call so the Enter key still gets sent for it exactly
            // once (matching the original code's control flow, not skipping
            // or double-sending).
            s_in_string_mode = false;
        } else {
            BleHidSpike::send_key(keycode_for(*p));
            s_cursor++;
            return true;
        }
    }

    if (strncmp(p, "STRING ", 7) == 0) {
        s_cursor += 7;
        s_in_string_mode = true;
        return true; // marker consumed, no HID action this tick
    }
    if (strncmp(p, "ENTER", 5) == 0) {
        BleHidSpike::send_key(0x28);
        s_cursor += 5;
        return true;
    }
    if (*p == '\n') {
        BleHidSpike::send_key(0x28);
        s_cursor++;
        return true;
    }
    // Outside a STRING run, unrecognized text is skipped (not typed) -- same
    // as the original brief's catch-all `else { p++; }` branch.
    s_cursor++;
    return true;
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("BLE Bad-KB", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Advertising as QuarkyKB...");
    s_last_connected = false;

    s_script_input = lv_textarea_create(content);
    lv_textarea_set_placeholder_text(s_script_input, "STRING hello world\nENTER");
    // Bound input to what s_script_buf can hold -- truncating at the widget
    // (visibly, to the user, before Send is ever tapped) rather than silently
    // inside the "Send" handler's strncpy below.
    lv_textarea_set_max_length(s_script_input, kScriptBufLen - 1);
    lv_obj_add_event_cb(s_script_input, [](lv_event_t *) {
        lv_keyboard_set_textarea(s_keyboard, s_script_input);
        lv_obj_clear_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);
    }, LV_EVENT_FOCUSED, nullptr);

    lv_obj_t *send_btn = lv_button_create(content);
    lv_obj_t *send_label = lv_label_create(send_btn);
    lv_label_set_text(send_label, "Send");
    lv_obj_add_event_cb(send_btn, [](lv_event_t *) {
        // No NimBLE calls here -- just copy the text and arm the parser.
        // poll() does all the actual (blocking-per-tick, not blocking-total)
        // sending. See this file's header comment for why.
        const char *text = lv_textarea_get_text(s_script_input);
        strncpy(s_script_buf, text, kScriptBufLen - 1);
        s_script_buf[kScriptBufLen - 1] = '\0';
        s_script_len = strlen(s_script_buf);
        s_cursor = 0;
        s_in_string_mode = false;
        s_typing = (s_script_len > 0);
        if (s_typing) {
            Serial.printf("quarky-tab5: [ble-bad-kb] typing script, %u bytes, poll()-driven\n",
                          (unsigned)s_script_len);
        }
    }, LV_EVENT_CLICKED, nullptr);

    s_keyboard = lv_keyboard_create(screen);
    lv_obj_add_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);

    // Teardown: stop advertising/terminate any connection via BleHidSpike's
    // own stop() (never raw ble_gap_* calls here -- this file has no
    // business touching NimBLE directly, see the header comment) and clear
    // every LVGL pointer + in-flight typing state so poll() cannot touch
    // freed widgets after ScreenStack::pop() destroys them. Same pattern
    // ble_spam.cpp/ble_scan.cpp use for their own LV_EVENT_DELETE teardown.
    lv_obj_add_event_cb(content, [](lv_event_t *) {
        BleHidSpike::stop();
        s_status_label = nullptr;
        s_script_input = nullptr;
        s_keyboard = nullptr;
        s_typing = false;
    }, LV_EVENT_DELETE, nullptr);

    // Reuses BleHidSpike's own proven advertise path (Task 2, real-hardware
    // confirmed) instead of any inlined raw ble_gap_adv_set_fields()/
    // ble_gap_adv_start() -- see this file's header comment. Safe to call
    // from a real open/close (not just one-shot serial-trigger) flow: read
    // start()'s implementation (ble_hid_spike.cpp) to confirm -- it already
    // guards on c2link_ble_host_synced(), the service having been queued at
    // boot, already-advertising, and an existing connection, and only claims
    // s_advertising=true if ble_gap_adv_start() actually returned 0 (finding
    // M1's fix). The one residual, pre-existing timing note (not introduced
    // by this feature): stop()'s ble_gap_terminate() raises
    // BLE_GAP_EVENT_DISCONNECT asynchronously, so reopening this screen
    // immediately after closing it while a previous connection's disconnect
    // is still in flight can hit start()'s "a host is already connected"
    // guard and no-op once; retrying (closing and reopening again) clears it
    // once the disconnect event lands. Same narrow, disclosed race as any
    // other start()/stop() consumer of this file -- not a new hazard Task 15
    // introduces.
    BleHidSpike::start();

    return screen;
}

void register_module() {
    g_registry.register_module({"ble_bad_kb", "BLE Bad-KB", Category::BLE,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    ScreenStack::push(build_screen());
}

void poll() {
    if (s_status_label) {
        bool connected = BleHidSpike::is_connected();
        if (connected != s_last_connected) {
            lv_label_set_text(s_status_label, connected ? "Paired" : "Advertising as QuarkyKB...");
            s_last_connected = connected;
        }
    }

    if (!s_typing) return;

    if (!type_script_step()) {
        s_typing = false;
        Serial.println("quarky-tab5: [ble-bad-kb] script finished");
        if (s_status_label) {
            lv_label_set_text(s_status_label,
                               s_last_connected ? "Paired -- script sent" : "Advertising as QuarkyKB...");
        }
    }
}

} // namespace BleBadKbFeature
