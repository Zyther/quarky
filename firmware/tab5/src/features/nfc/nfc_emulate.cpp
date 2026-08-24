#include "nfc_emulate.h"

#include "st25r3916_driver.h"
#include "../../hal/nfc_pn532.h" // nfc_release_external_i2c() -- GPIO53 arbiter

#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"

#include <lvgl.h>
#include <Arduino.h>
#include <cstdio>

namespace NfcEmulate {

namespace {

// Screen-lifetime state machine, same two-phase "arm on the next poll() tick,
// never inside a click handler" shape nfc_read.cpp's ScanState/run_bring_up()
// already uses, and for the same reason: listen_start()'s ~15-byte PT-memory
// write plus ~10 register writes plus an osc_ok poll is a one-shot cost that
// must not run synchronously inside start()'s caller (nfc_tag_library_ui.cpp's
// "Emulate" button click handler).
enum class EmulateState : uint8_t {
    kArming,        // screen just opened; the next poll() tick arms Listen Mode
    kWaiting,       // armed; St25r3916::listen_poll() reports kIdle
    kSelected,      // St25r3916::listen_poll() reports kSelected
    kFailed,        // listen_start() failed, or listen_poll() reported
                    // kHardwareError -- latched, no retry until re-entry
};

EmulateState s_state = EmulateState::kArming;
NfcCommon::TagInfo s_tag{};

lv_obj_t *s_status_label = nullptr;
lv_obj_t *s_tag_label = nullptr;

bool s_unit_armed = false; // true once listen_start() has succeeded, so
                           // teardown() knows whether it owes the chip a
                           // listen_stop() -- same bookkeeping nfc_read.cpp's
                           // s_unit_ready serves for the reader path.

void set_status(const char *text) {
    if (s_status_label != nullptr) {
        lv_label_set_text(s_status_label, text);
    }
}

void run_arm() {
    St25r3916::ListenConfig cfg{};
    cfg.uid_len = s_tag.uid_len;
    for (uint8_t i = 0; i < sizeof(cfg.uid); i++) {
        cfg.uid[i] = s_tag.uid[i];
    }
    cfg.atqa[0] = s_tag.atqa[0];
    cfg.atqa[1] = s_tag.atqa[1];
    // A {0,0} ATQA means nfc_common.h's "not captured" sentinel (true for
    // every tag saved via the RFID2 path -- see that header's own comment).
    // Fall back to the ISO14443-3-consistent default for this UID length
    // (bit 6 = "UID size double", i.e. 7 bytes; the low nibble's anticoll
    // bits are otherwise 0) rather than arming with an ATQA no real tag of
    // this UID length would ever report.
    if (cfg.atqa[0] == 0 && cfg.atqa[1] == 0) {
        cfg.atqa[0] = (cfg.uid_len == 7U) ? 0x44U : 0x04U;
        cfg.atqa[1] = 0x00U;
    }
    cfg.sak = s_tag.sak;

    if (!St25r3916::listen_start(cfg)) {
        s_state = EmulateState::kFailed;
        s_unit_armed = false;
        set_status("Could not arm Listen Mode -- check the NFC unit is\n"
                   "plugged into PORT.A. Leave and re-enter to retry.");
        Serial.println("quarky-tab5: [nfc-emulate] listen_start() failed -- "
                       "latched (no retry)");
        return;
    }

    s_unit_armed = true;
    s_state = EmulateState::kWaiting;
    set_status("Waiting for a reader...");
}

void teardown() {
    s_status_label = nullptr;
    s_tag_label = nullptr;
    s_state = EmulateState::kArming;

    if (s_unit_armed) {
        St25r3916::listen_stop();
    }
    s_unit_armed = false;

    // Same GPIO53-arbiter release every other NFC-unit screen's teardown()
    // does (nfc_read.cpp, nfc_emv_read.cpp, ...) -- unconditional, since
    // St25r3916::init() (called from inside listen_start()) may have already
    // claimed the pin even on a failed arm attempt.
    nfc_release_external_i2c();
}

lv_obj_t *build_screen() {
    s_state = EmulateState::kArming;
    s_unit_armed = false;

    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("Emulate Tag", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_tag_label = lv_label_create(content);
    char uid_str[64];
    NfcCommon::format_uid(s_tag.uid, s_tag.uid_len, uid_str, sizeof(uid_str));
    char buf[160];
    std::snprintf(buf, sizeof(buf), "%s\nUID: %s\nSAK %02X", s_tag.type_name,
                  uid_str, (unsigned)s_tag.sak);
    lv_label_set_text(s_tag_label, buf);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Arming...");

    lv_obj_add_event_cb(content, [](lv_event_t *) { teardown(); },
                        LV_EVENT_DELETE, nullptr);

    return screen;
}

} // namespace

void start(const NfcCommon::TagInfo &tag) {
    s_tag = tag;
    ScreenStack::push(build_screen());
}

void poll() {
    // No screen -> nothing to do, same guard every other feature's poll()
    // uses to stay free when its own screen isn't open.
    if (s_status_label == nullptr) {
        return;
    }

    if (s_state == EmulateState::kArming) {
        run_arm();
        return;
    }
    if (s_state == EmulateState::kFailed) {
        return; // latched; re-enter the screen to retry
    }

    // One bounded tick -- St25r3916::listen_poll() is three I2C register reads
    // on an ordinary tick, plus (only on a reader-field edge, i.e. at most
    // twice per reader presentation) the handful of register writes and direct
    // commands that drive the chip's POWER_OFF <-> IDLE state entry. So this is
    // safe to call every loop() iteration without risking the ~5 s
    // task-watchdog window this project has already been bitten by twice
    // (hal/ir_unit.h's and hal/storage_sd.cpp's own header comments): the only
    // blocking wait it can reach at all is the same bounded 10 ms
    // oscillator-stable poll field_on() uses, and only if something had
    // cleared OP_CONTROL.en behind our back.
    const St25r3916::ListenState st = St25r3916::listen_poll();
    switch (st) {
        case St25r3916::ListenState::kIdle:
            if (s_state != EmulateState::kWaiting) {
                s_state = EmulateState::kWaiting;
                set_status("Waiting for a reader...");
            }
            break;
        case St25r3916::ListenState::kSelected:
            if (s_state != EmulateState::kSelected) {
                s_state = EmulateState::kSelected;
                set_status("Reader detected this tag's UID!\n"
                          "(UID/SAK/ATQA only -- no data exchange yet.)");
            }
            break;
        case St25r3916::ListenState::kHardwareError:
            s_state = EmulateState::kFailed;
            s_unit_armed = false; // the chip already stopped answering;
                                  // nothing to un-arm on teardown
            set_status("NFC unit stopped responding.\n"
                      "Leave and re-enter this screen to retry.");
            Serial.println("quarky-tab5: [nfc-emulate] listen_poll() reported "
                           "a hardware error -- latched (no retry)");
            break;
        case St25r3916::ListenState::kNotArmed:
        default:
            // Should not happen while s_unit_armed is true; nothing to do.
            break;
    }
}

} // namespace NfcEmulate
