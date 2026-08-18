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
static lv_obj_t *s_name_input = nullptr;
static lv_obj_t *s_start_btn = nullptr;
static lv_obj_t *s_script_input = nullptr;
static lv_obj_t *s_keyboard = nullptr;
static bool s_last_connected = false;
// True iff BleHidSpike::start() reported real success (see the Start
// button's click handler, below). Task 15 review-round finding (Important):
// build_screen() used to set the status label to "Advertising as
// QuarkyKB..." UNCONDITIONALLY, before even checking start()'s outcome --
// but start() has four real early-return failure paths (NimBLE host not
// synced, e.g. this project's own "radios disabled for this boot" mode; the
// HID service never got queued; already advertising; a host is already
// connected from a still-live prior session, see the reopen-race note at the
// start() call site below). Two of those are reachable in normal use, and
// the label was lying in both -- exactly the "false negative reads as
// success" outcome ble_hid_spike.cpp's own send_key() "refuse rather than
// lie" guard exists to prevent at the transport layer, reintroduced one
// level up in this UI. update_status_label() now derives the label from
// s_last_connected/s_advertise_ok together instead of asserting one fixed
// string.
static bool s_advertise_ok = false;
// True from the moment "Start" is first tapped this screen-open, until
// teardown. Distinguishes "never attempted" (a fresh screen open, before the
// device-name textbox has even been used) from "attempted and failed" for
// update_status_label() -- both are !s_last_connected && !s_advertise_ok,
// but they need different text: a fresh open showing "Not paired -- cannot
// send" before the user has done anything would read as an unexplained
// error, not an invitation to type a name and tap Start.
static bool s_start_attempted = false;

// Single source of truth for the status label's text, called once at screen
// build (before Start has ever been tapped) and again after every Start tap
// and on every connection-state change from poll(). Connected always wins (a
// host being connected is true regardless of why start() itself returned
// false -- see the "already connected" case in start()'s own comment: that
// failure path means a PRIOR session's connection is still live, which is
// still a real, working pairing from this label's point of view).
static void update_status_label() {
    if (!s_status_label) return;
    if (s_last_connected) {
        lv_label_set_text(s_status_label, "Paired");
    } else if (s_advertise_ok) {
        char buf[48]; // "Advertising as \"" + up to kMaxDeviceNameLen (18)
                      // name bytes + "\"..." + '\0' comfortably fits
        snprintf(buf, sizeof(buf), "Advertising as \"%s\"...", BleHidSpike::device_name());
        lv_label_set_text(s_status_label, buf);
    } else if (!s_start_attempted) {
        lv_label_set_text(s_status_label, "Enter a name and tap Start");
    } else {
        lv_label_set_text(s_status_label, "Not paired -- cannot send");
    }
}

// Review finding (2026-08-18): re-tapping Start after a successful start
// used to silently lie -- BleHidSpike::start() early-returns true on its
// "already advertising" guard without calling start_advertising() again, so
// a name typed and Start-tapped a second time updated s_device_name (and
// therefore this file's own status label, which reads it back via
// BleHidSpike::device_name()) while the radio kept broadcasting the FIRST
// name. Disabling both widgets whenever there is nothing left for Start to
// usefully do -- genuinely advertising, or already paired -- removes the
// only way to reach that lie, rather than trying to make a second start()
// call actually restart with the new name (a bigger change than this
// feature's scope).
//
// Re-review finding (2026-08-18): the first version of this fix only ever
// LOCKED, never unlocked -- so the reopen-race build-time lock
// (s_last_connected true at build_screen()) became a permanent dead end the
// moment that prior session's connection actually dropped: poll()'s
// connection-state-change branch would correctly flip the label back to
// "Enter a name and tap Start", but the widgets stayed disabled forever,
// with no action left that could ever re-enable them short of closing and
// reopening the screen. Made symmetric instead -- derives lock state fresh
// from real current state (s_last_connected/s_advertise_ok) every time it's
// called, so it can un-lock just as correctly as it locks. Called from
// build_screen() (initial state), the Start handler (after a tap), and
// poll()'s connection-change branch (a disconnect while the screen stays
// open) -- every place update_status_label() already runs, since "is there
// anything for Start to usefully do" and "what does the label say" are the
// same underlying question.
static void sync_name_controls_lock() {
    if (!s_name_input || !s_start_btn) return;
    if (s_last_connected || s_advertise_ok) {
        lv_obj_add_state(s_name_input, LV_STATE_DISABLED);
        lv_obj_add_state(s_start_btn, LV_STATE_DISABLED);
        // Re-review finding (2026-08-18): LV_STATE_DISABLED alone leaves a
        // keyboard that was linked to s_name_input BEFORE this lock fired
        // still delivering keystrokes to it -- lv_keyboard's own button
        // handler calls lv_textarea_add_text() directly on whatever
        // textarea it's linked to, with no LV_STATE_DISABLED check anywhere
        // in lv_textarea itself (unlike the FOCUSED-handler guard above,
        // which only stops a NEW link from being made, not an existing
        // one). Explicitly unlink and hide the keyboard here too, but only
        // if it's currently linked to s_name_input specifically -- locking
        // the name controls must not steal focus from the script textarea
        // if that's what the user is actively typing into.
        if (s_keyboard && lv_keyboard_get_textarea(s_keyboard) == s_name_input) {
            lv_keyboard_set_textarea(s_keyboard, nullptr);
            lv_obj_add_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);
        }
    } else {
        lv_obj_clear_state(s_name_input, LV_STATE_DISABLED);
        lv_obj_clear_state(s_start_btn, LV_STATE_DISABLED);
    }
}

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
    lv_label_set_text(s_status_label, "..."); // corrected below, right after
                                               // s_last_connected/
                                               // s_start_attempted are set --
                                               // never asserted as
                                               // "Advertising" ahead of a
                                               // real start() result (see
                                               // s_advertise_ok's comment)
    s_last_connected = BleHidSpike::is_connected(); // a prior session's
                                                      // connection can already
                                                      // be live here (the
                                                      // reopen-race case) --
                                                      // check reality, don't
                                                      // assume false
    s_advertise_ok = BleHidSpike::is_advertising(); // same reasoning as
                                                      // s_last_connected just
                                                      // above -- BleHidSpike
                                                      // can already be
                                                      // advertising here too
                                                      // (main.cpp's 'h'
                                                      // serial-debug trigger
                                                      // bypasses this screen
                                                      // entirely), so check
                                                      // reality rather than
                                                      // assume a fresh false
    s_start_attempted = false;

    // Device-name input -- Task "device-name textbox" (BLE Bad-KB UI): the
    // name this HID device broadcasts under, previously a fixed "QuarkyKB"
    // baked into ble_hid_spike.cpp. Placeholder (not pre-filled text) shows
    // the real default so an empty submit still does something sensible --
    // BleHidSpike::set_device_name() falls back to "QuarkyKB" on an empty
    // name for exactly that reason. max_length is BleHidSpike's own
    // kMaxDeviceNameLen, the real legacy-advertisement byte budget (see its
    // comment in ble_hid_spike.h) -- truncating visibly at the widget rather
    // than silently wherever the name is next used, same rationale as the
    // script textarea's own max_length just below.
    s_name_input = lv_textarea_create(content);
    lv_textarea_set_one_line(s_name_input, true);
    lv_textarea_set_placeholder_text(s_name_input, "QuarkyKB");
    lv_textarea_set_max_length(s_name_input, BleHidSpike::kMaxDeviceNameLen);
    lv_obj_add_event_cb(s_name_input, [](lv_event_t *) {
        // Re-review finding (2026-08-18): LV_STATE_DISABLED (see
        // sync_name_controls_lock()) does not stop an lv_textarea from
        // taking focus or accepting typed input in this LVGL version --
        // only LV_EVENT_CLICKED delivery to buttons is gated on it. Without
        // this explicit check, a "locked" name field would still visibly
        // accept edits and pop the keyboard, implying the change does
        // something even though Start (the only thing that reads this text)
        // is genuinely unclickable while locked.
        if (lv_obj_has_state(s_name_input, LV_STATE_DISABLED)) return;
        lv_keyboard_set_textarea(s_keyboard, s_name_input);
        lv_obj_clear_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);
    }, LV_EVENT_FOCUSED, nullptr);

    s_start_btn = lv_button_create(content);
    lv_obj_t *start_label = lv_label_create(s_start_btn);
    lv_label_set_text(start_label, "Start");
    lv_obj_add_event_cb(s_start_btn, [](lv_event_t *) {
        // Empty text falls through to BleHidSpike::set_device_name()'s own
        // null/empty -> "QuarkyKB" fallback (see its comment in
        // ble_hid_spike.h) -- this handler doesn't need its own default
        // string, just don't call set_device_name() with a dangling read of
        // an empty buffer's contents (lv_textarea_get_text() never returns
        // nullptr for a live textarea, so passing it straight through is
        // safe either way).
        BleHidSpike::set_device_name(lv_textarea_get_text(s_name_input));
        s_start_attempted = true;

        // Reuses BleHidSpike's own proven advertise path (Task 2,
        // real-hardware confirmed) instead of any inlined raw
        // ble_gap_adv_set_fields()/ble_gap_adv_start() -- see this file's
        // header comment. Bool return is checked (Task 15 review-round
        // finding, Important): start() has four real early-return failure
        // paths (host not synced, service never queued, already
        // advertising, host already connected), and asserting "Advertising"
        // without checking would repeat the exact "false negative reads as
        // success" bug that finding fixed once already. One residual,
        // pre-existing timing note: stop()'s ble_gap_terminate() (previous
        // session's teardown) raises BLE_GAP_EVENT_DISCONNECT
        // asynchronously, so tapping Start again immediately after a very
        // recent disconnect can hit the "already connected" guard and
        // return false here -- but s_last_connected (set at screen-build,
        // above) already reflects that a host IS in fact still connected in
        // that exact case, so update_status_label() below correctly shows
        // "Paired" rather than this call's own false result.
        s_advertise_ok = BleHidSpike::start();
        Serial.printf("quarky-tab5: [ble-bad-kb] Start tapped, name=\"%s\", "
                      "advertise_ok=%d\n", BleHidSpike::device_name(), (int)s_advertise_ok);
        update_status_label();
        // Lock out further name edits/retaps once genuinely advertising --
        // see sync_name_controls_lock()'s comment for why a second tap would
        // lie otherwise. Left enabled on failure so the user can retry (e.g.
        // after fixing whatever start() rejected).
        sync_name_controls_lock();
    }, LV_EVENT_CLICKED, nullptr);

    // Reopen-race case (see s_last_connected's comment above): a prior
    // session's connection can already be live when this screen opens fresh.
    // Tapping Start there would hit start()'s own "already connected"
    // early-return and do nothing useful -- lock it out up front rather than
    // offering an action with no effect. (If that connection later drops
    // while this screen stays open, poll()'s connection-change branch below
    // calls sync_name_controls_lock() again and correctly unlocks it.)
    sync_name_controls_lock();

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
        // Review-round finding (Important): tapping Send while genuinely
        // unconnected used to drain the whole script through send_key()'s
        // deliberately-silent "not connected" early return -- no UI change,
        // no serial output, just the status label getting rewritten to the
        // same string it already showed. Same "false negative reads as
        // success" class as the label bug above, one layer further in.
        // Refuse up front instead, with real feedback, rather than silently
        // no-opping through the whole script one character at a time.
        if (!BleHidSpike::is_connected()) {
            Serial.println("quarky-tab5: [ble-bad-kb] Send tapped while not paired -- ignoring");
            update_status_label(); // re-assert the true current state, in
                                    // case the user tapped Send before
                                    // noticing the label
            return;
        }

        const char *text = lv_textarea_get_text(s_script_input);
        if (text == nullptr || text[0] == '\0') {
            // Minimal feedback for the empty-script case rather than silence
            // (the original code would set s_typing=false and do nothing
            // visible at all).
            Serial.println("quarky-tab5: [ble-bad-kb] Send tapped with an empty script -- nothing to type");
            return;
        }

        // No NimBLE calls here -- just copy the text and arm the parser.
        // poll() does all the actual (blocking-per-tick, not blocking-total)
        // sending. See this file's header comment for why.
        strncpy(s_script_buf, text, kScriptBufLen - 1);
        s_script_buf[kScriptBufLen - 1] = '\0';
        s_script_len = strlen(s_script_buf);
        s_cursor = 0;
        s_in_string_mode = false;
        s_typing = true;
        Serial.printf("quarky-tab5: [ble-bad-kb] typing script, %u bytes, poll()-driven\n",
                      (unsigned)s_script_len);
    }, LV_EVENT_CLICKED, nullptr);

    s_keyboard = lv_keyboard_create(screen);
    lv_obj_add_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);

    // Teardown: stop advertising/terminate any connection via BleHidSpike's
    // own stop() (never raw ble_gap_* calls here -- this file has no
    // business touching NimBLE directly, see the header comment) and clear
    // every LVGL pointer + in-flight typing state so poll() cannot touch
    // freed widgets after ScreenStack::pop() destroys them. Same pattern
    // ble_spam.cpp/ble_scan.cpp use for their own LV_EVENT_DELETE teardown.
    //
    // Unconditional, including when this screen-open never tapped Start at
    // all (a real, now-reachable case with Start opt-in rather than
    // build_screen() always calling start()): stop() itself is safe to call
    // whether or not a matching start() ever ran -- it only touches the
    // radio's advertising slot if it actually took it (guarded on
    // s_took_adv_slot, see ble_hid_spike.cpp's stop()), so an unmatched call
    // here cannot stop c2link_ble's own advertisement.
    lv_obj_add_event_cb(content, [](lv_event_t *) {
        BleHidSpike::stop();
        s_status_label = nullptr;
        s_name_input = nullptr;
        s_start_btn = nullptr;
        s_script_input = nullptr;
        s_keyboard = nullptr;
        s_typing = false;
        s_advertise_ok = false;
        s_start_attempted = false;
    }, LV_EVENT_DELETE, nullptr);

    // Device-name-textbox redesign: BleHidSpike::start() is no longer called
    // here unconditionally -- it now only runs from the Start button's own
    // LV_EVENT_CLICKED handler above, once a name is available to hand it.
    // This just renders whichever initial state applies (a still-live prior
    // connection from the reopen-race case, or the fresh "enter a name and
    // tap Start" prompt) via the same single-source-of-truth label function
    // every later state change uses too.
    update_status_label();

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
    bool connected = BleHidSpike::is_connected();
    if (connected != s_last_connected) {
        s_last_connected = connected;
        if (s_status_label) {
            update_status_label();
            // Re-review finding (2026-08-18): a disconnect that drops
            // s_last_connected back to false while this screen stays open
            // (e.g. the reopen-race build-time lock's connection finally
            // ending) needs the widgets re-synced too, not just the label --
            // see sync_name_controls_lock()'s comment for the dead-end this
            // closes.
            sync_name_controls_lock();
        }
    }

    // Reconciles s_advertise_ok against BleHidSpike's own real advertising
    // state every tick, the same way s_last_connected is reconciled against
    // is_connected() just above -- not only set once from the Start button's
    // own call site. Closes a real gap (re-review finding, 2026-08-18):
    // main.cpp's 'h' serial-debug trigger calls BleHidSpike::start()
    // directly, bypassing this screen's Start button entirely, so a
    // QUARKY_SERIAL_DEBUG build where 'h' fires while this screen happens to
    // be open would otherwise leave the label/lock state stale.
    bool advertising = BleHidSpike::is_advertising();
    if (advertising != s_advertise_ok) {
        s_advertise_ok = advertising;
        if (s_status_label) {
            update_status_label();
            sync_name_controls_lock();
        }
    }

    if (!s_typing) return;

    if (!connected) {
        // Host disconnected mid-script. Without this, the remaining buffer
        // would keep draining through send_key()'s silent not-connected
        // no-op, one wasted poll() tick per character, with the label above
        // already showing the truth but the "Send" action itself never
        // visibly concluding. Stop here instead -- consistent with this
        // review round's broader theme of the UI reflecting real transport
        // state rather than silently doing nothing.
        s_typing = false;
        Serial.println("quarky-tab5: [ble-bad-kb] host disconnected mid-script -- aborting remaining keystrokes");
        return;
    }

    if (!type_script_step()) {
        s_typing = false;
        Serial.println("quarky-tab5: [ble-bad-kb] script finished");
        if (s_status_label) {
            // Direct set (not update_status_label()) since this is a
            // one-time "just finished" message distinct from the label's
            // steady-state text -- only meaningful in the connected case;
            // the disconnected case falls back to the same truthful
            // steady-state label update_status_label() would produce.
            lv_label_set_text(s_status_label,
                               s_last_connected ? "Paired -- script sent" : "Not paired -- cannot send");
        }
    }
}

} // namespace BleBadKbFeature
