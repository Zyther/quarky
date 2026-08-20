#include "rf433_bruteforce.h"
#include "rf433_replay.h"
#include "rf433_scan.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <Arduino.h>
#include <cstdint>
#include <cstdio>
#include <cstring>

extern FeatureRegistry g_registry;

namespace Rf433Bruteforce {
namespace {

// ── Ported timing table ─────────────────────────────────────────────────
// ~/src/firmware/src/modules/rf/rf_bruteforce.h. Struct shape: h:6-13.
// Table: h:17-25, ported verbatim (name, bits, zero/one/pilot/stop duration
// pairs). Timing convention (h:16, preserved here): positive = HIGH,
// negative = LOW, magnitude in microseconds. {0, 0} for pilot/stop means
// "this protocol has none" (see rf_bruteforce.cpp:66,79 -- both guard on
// `if (proto.pilot[0] || proto.pilot[1])` / `proto.stop[...]`, not on a
// separate has-pilot/has-stop flag).
struct BruteProtocol {
    const char *name;
    int bits;
    int zero[2];
    int one[2];
    int pilot[2];
    int stop[2];
};

// clang-format off
constexpr BruteProtocol kBruteProtocols[] = {
    // name               bits  zero           one           pilot            stop
    {"Came 12bit",       12, {-320, 640},  {-640, 320},  {-11520, 320}, {0, 0}       },
    {"Nice 12bit",       12, {-700, 1400}, {-1400, 700}, {-25200, 700}, {0, 0}       },
    {"Ansonic 12bit",    12, {-1111, 555}, {-555, 1111}, {-19425, 555}, {0, 0}       },
    {"Holtek 12bit",     12, {-870, 430},  {-430, 870},  {-15480, 430}, {0, 0}       },
    {"Linear 10bit",     10, {500, -1500}, {1500, -500}, {0, 0},        {500, -21500}},
    // KNOWN-SUSPECT ENCODING (flagged in task-8-report.md's "Cross-donor
    // corroboration" section, not fixed there per the controller's explicit
    // "don't let it become a rabbit hole" -- caveat added here per the Task 8
    // review round instead): the other five rows above match their
    // independently-sourced Task 7/UniGeek-Flipper `SubGhzDecoders.cpp`
    // te_short/te_long constants exactly. This row does NOT -- it is
    // byte-for-byte identical to the Holtek row just above it (same 430/870
    // pulse pair, same shape), while Task 7's real decode_chamberlain()
    // (SubGhzDecoders.cpp:1174-1228) expects a structurally different 4-bit-
    // symbol DIP-code encoding (each symbol one of 0b0111/0b0011/0b0001 at
    // multiples of a 1000us base unit), not a positional MSB-first binary
    // code at all. Bruce's own donor table (rf_bruteforce.h:17-25) appears to
    // have mislabeled a second Holtek-shaped entry as "Chamberlain" rather
    // than encoding real Chamberlain/DIP-switch timing. A candidate this
    // module generates for this row will very likely NOT be recognized by
    // Task 7's decoder, and may not match a real Chamberlain device's actual
    // wire format either -- hence "(unverified encoding)" appended to the
    // dropdown label below, so a user selecting it sees the caveat before
    // running a sweep against real hardware.
    {"Chamberlain 9bit (unverified encoding)", 9, {-870, 430}, {-430, 870}, {0, 0}, {-3000, 1000}},
};
// clang-format on
constexpr int kBruteProtocolCount = sizeof(kBruteProtocols) / sizeof(kBruteProtocols[0]);

// Donor default protocol index (rf_bruteforce.cpp:5, "// Default: Nice") --
// kept as this port's initial dropdown selection too, so a first-time user
// sees the same starting point the donor tool did.
constexpr int kDefaultProtocolIdx = 1;

// Donor's repeat-per-code count is user-configurable 1-5 via rf_brute_repeats()
// (rf_bruteforce.cpp:28-34), defaulting to 1 (rf_bruteforce.cpp:6). This port
// fixes it at the donor's own default rather than exposing a third UI control
// -- Task 8's brief scopes Step 2 to "progress bar + Stop button" only, and a
// fixed value of 1 is a real, cited donor behavior (its default), not an
// invented simplification. At up to 12 bits/protocol, one repeat is at most
// ~26 pulses (pilot + 12*2 bits + stop) -- see build_candidate()'s comment on
// why this can never approach kMaxEdgesPerSignal (512) even so.
constexpr int kRepeatsPerCode = 1;

// ── Candidate signal synthesis ──────────────────────────────────────────

// Running state for converting an ordered (level, duration) pulse stream --
// the same stream Bruce's sendPulse() (rf_bruteforce.cpp:36-44) would
// bit-bang directly -- into the EdgeSample{timestamp_us, level-after-
// transition} shape rf433_common.h:28-31 documents and Rf433Replay's
// transmit_task() (rf433_replay.cpp:152-171) expects.
struct SignalBuilder {
    Rf433Scan::CapturedSignal *sig;
    uint32_t next_timestamp_us = 0;
    bool have_first = false;
};

// Appends one pulse (duration_us: positive = HIGH, negative = LOW, same
// convention as kBruteProtocols' fields) to the signal being built.
//
// Mirrors sendPulse()'s own no-op-on-zero behavior (rf_bruteforce.cpp:37,40
// only act "if (duration > 0)" / "if (duration < 0)"): a duration of exactly
// 0 emits nothing, which is what makes a protocol's {0, 0} pilot/stop pair
// (Came/Nice/Ansonic/Holtek's pilot; Came/Nice/Ansonic/Holtek's own stop is
// also {0,0}) correctly contribute no pulses at all when both callers below
// guard on `pilot[0] || pilot[1]` / `stop[0] || stop[1]` exactly like
// rf_bruteforce.cpp:66 and :79 do.
//
// Like every real CapturedSignal (rf433_scan.cpp's finalize_burst()) and
// unlike Bruce's direct digitalWrite()+delayMicroseconds() loop, the LAST
// pulse appended to a signal has its own hold-time left unrepresented in
// edges[]: an EdgeSample only records the level a transition moves TO, and
// there is no further edge after the last one to measure how long it was
// held before the burst ended. This is not a limitation introduced by this
// port -- it is the same format contract every real RF433 capture already
// has (see rf433_protocol_decode.cpp's build_durations() comment on exactly
// this) -- so a synthesized signal built to the same contract can go
// through the identical Rf433Replay::transmit() path as a real one with no
// special-casing.
void append_pulse(SignalBuilder &b, int32_t duration_us) {
    if (duration_us == 0) return;
    if (b.sig->edge_count >= Rf433Scan::kMaxEdgesPerSignal) return; // defensive; see
                                                                      // build_candidate()'s
                                                                      // comment for why this
                                                                      // never actually fires

    bool level = duration_us > 0;
    uint32_t magnitude = level ? (uint32_t)duration_us : (uint32_t)(-duration_us);

    if (!b.have_first) {
        // First pulse of the whole signal sets the starting level at t=0;
        // its own hold time becomes the timestamp of the edge that ends it
        // (applied below, on the NEXT call).
        b.sig->edges[0].timestamp_us = 0;
        b.sig->edges[0].level = level;
        b.sig->edge_count = 1;
        b.have_first = true;
        b.next_timestamp_us = magnitude;
        return;
    }

    size_t idx = b.sig->edge_count;
    b.sig->edges[idx].timestamp_us = b.next_timestamp_us;
    b.sig->edges[idx].level = level;
    b.sig->edge_count = idx + 1;
    b.next_timestamp_us += magnitude;
}

// Critical fix (Task 8 review round, 2026-08-20): appends one more
// terminating edge, forcing LOW, at b.next_timestamp_us -- i.e. exactly
// where the pulse most recently appended via append_pulse() would end.
//
// WHY this is necessary, not cosmetic: append_pulse()'s own comment
// (above) documents that the LAST pulse appended to a signal has its hold
// time left unrepresented in edges[] -- an EdgeSample only records the
// level a transition moves TO, so there is no edge recording how long the
// final level was held. Rf433Replay::transmit_task() (rf433_replay.cpp:
// 152-171) matches that contract exactly: it writes edges[i].level, then
// delays for (edges[i+1].timestamp - edges[i].timestamp), then writes
// edges[i+1].level -- so the hold time AFTER the last edge is never
// delayed at all; transmit_task() forces the pin LOW immediately after
// writing the last edge's level.
//
// For a REAL capture (Task 5) this is harmless: the last recorded edge is
// the transition INTO Task 5's own >25ms trailing silence gap
// (kBurstGapThresholdUs), so dropping that hold time doesn't corrupt any
// real protocol data -- it only shortens silence nobody was measuring.
//
// For a SYNTHESIZED bruteforce candidate, the last append_pulse() call
// inside build_candidate()'s loop is NOT silence -- it's the final data
// bit's meaningful HIGH/LOW pulse (Came/Nice/Ansonic/Holtek/Linear/
// Chamberlain all encode real data right up to the stop symbol, when they
// have one), so dropping its hold time transmitted a corrupted final
// pulse on every single candidate this module generated. Fix: append one
// more edge -- forced LOW, with no semantic meaning of its own beyond
// "settle low" -- so the true last DATA pulse (the one before this call)
// gets a real, correctly-timed edge marking the end of ITS hold time.
// This new terminator edge becomes the signal's new "last edge", and its
// own (synthetic, don't-care) hold time is the one now silently dropped
// by transmit_task() -- exactly as intended.
void append_terminator(SignalBuilder &b) {
    if (!b.have_first) return; // no pulses were ever appended (shouldn't happen: every
                                // protocol here has bits > 0)
    if (b.sig->edge_count >= Rf433Scan::kMaxEdgesPerSignal) return; // defensive; see
                                                                      // build_candidate()'s
                                                                      // comment -- never fires

    size_t idx = b.sig->edge_count;
    b.sig->edges[idx].timestamp_us = b.next_timestamp_us;
    b.sig->edges[idx].level = false; // force LOW; idle-safe, matches transmit_task()'s
                                      // own post-burst digitalWrite(..., LOW)
    b.sig->edge_count = idx + 1;
}

// Builds candidate `code`'s full pulse sequence for `proto` into `sig`,
// reusing this module's one static candidate buffer (see its declaration
// below). Real keyspace-walk/encoding sequence ported from rf_brute_start()'s
// inner loop, rf_bruteforce.cpp:63-83: pilot, then data bits MSB-first
// (each bit selects proto.one/proto.zero), then stop, repeated
// kRepeatsPerCode times.
//
// Edge-count ceiling, cited rather than assumed: the largest protocol here
// is 12 bits (Came/Nice/Ansonic/Holtek). One repeat is at most
// 2 (pilot) + 12*2 (bits) + 2 (stop) = 28 pulses/edges, plus one more for
// append_terminator()'s trailing settle-LOW edge (see its own comment) =
// 29 edges. At kRepeatsPerCode == 1 that is 29 edges total, nowhere near
// Rf433Scan::kMaxEdgesPerSignal (512) -- sig.truncated is unconditionally
// false below because this can never actually truncate.
void build_candidate(int protocol_idx, uint32_t code, Rf433Scan::CapturedSignal &sig) {
    sig.edge_count = 0;
    sig.captured_at_ms = millis();
    sig.capture_id = 0; // synthesized, not a real session capture -- no ring identity needed
    sig.truncated = false; // see this function's comment: 28 edges/repeat max, never near the 512 cap

    const BruteProtocol &proto = kBruteProtocols[protocol_idx];
    SignalBuilder b{&sig};

    for (int r = 0; r < kRepeatsPerCode; r++) {
        // Pilot/sync period. rf_bruteforce.cpp:65-69.
        if (proto.pilot[0] || proto.pilot[1]) {
            append_pulse(b, proto.pilot[0]);
            append_pulse(b, proto.pilot[1]);
        }

        // Data bits, MSB first. rf_bruteforce.cpp:71-76.
        for (int j = proto.bits - 1; j >= 0; j--) {
            const int *timings = ((code >> j) & 1) ? proto.one : proto.zero;
            append_pulse(b, timings[0]);
            append_pulse(b, timings[1]);
        }

        // Stop bit. rf_bruteforce.cpp:78-82.
        if (proto.stop[0] || proto.stop[1]) {
            append_pulse(b, proto.stop[0]);
            append_pulse(b, proto.stop[1]);
        }
    }

    // Critical fix (Task 8 review round): terminate once, after ALL repeats
    // (not once per repeat) -- see append_terminator()'s own comment for why
    // this is needed at all. Terminating inside the loop instead would
    // insert a spurious forced-LOW settle edge BETWEEN repeats, corrupting
    // back-to-back repeat framing for any future kRepeatsPerCode > 1; the
    // real last pulse needing its hold time restored is only the very last
    // one emitted across the whole candidate.
    append_terminator(b);
}

// ── Run state ────────────────────────────────────────────────────────────

bool s_active = false;
int s_protocol_idx = kDefaultProtocolIdx;
uint32_t s_current_code = 0;
uint32_t s_total_codes = 0;

enum class EndReason { kCompleted, kUserStopped, kTransmitFailed };

// One candidate signal, rebuilt fresh each tick. ~28 edges * 8 bytes
// (EdgeSample) plus bookkeeping is on the order of the same ~4KB static
// buffer main.cpp's rf433_dump_capture() already keeps on internal DRAM
// (`static Rf433Common::EdgeSample s_dump[512]`) -- three orders of
// magnitude below the 64KB kMaxCapturedSignals*sizeof(CapturedSignal)
// allocation that made rf433_scan.cpp's session buffer a real PSRAM-
// allocation requirement (see that file's s_signals comment). No
// heap_caps_malloc needed here for the same reason that buffer didn't need
// one either.
Rf433Scan::CapturedSignal s_candidate;

// UI widgets. Null when the screen isn't open (same convention as
// rf433_scan.cpp's s_status_label et al.) -- every UI-touching function
// below checks before writing.
lv_obj_t *s_status_label = nullptr;
lv_obj_t *s_progress_bar = nullptr;
lv_obj_t *s_protocol_dropdown = nullptr;
lv_obj_t *s_toggle_btn = nullptr;
lv_obj_t *s_toggle_label = nullptr;

void update_ui() {
    if (s_protocol_dropdown) {
        if (s_active) {
            lv_obj_add_state(s_protocol_dropdown, LV_STATE_DISABLED);
        } else {
            lv_obj_remove_state(s_protocol_dropdown, LV_STATE_DISABLED);
        }
    }
    if (s_toggle_label) {
        lv_label_set_text(s_toggle_label, s_active ? "Stop" : "Start");
    }
    if (!s_status_label) return;

    const BruteProtocol &proto = kBruteProtocols[s_protocol_idx];
    char buf[96];
    if (s_active) {
        std::snprintf(buf, sizeof(buf), "Running %s: code %lu / %lu",
                      proto.name, (unsigned long)s_current_code,
                      (unsigned long)s_total_codes);
    } else if (s_current_code == 0) {
        std::snprintf(buf, sizeof(buf), "Idle -- %s selected (%lu codes)",
                      proto.name, (unsigned long)(1u << proto.bits));
    } else {
        std::snprintf(buf, sizeof(buf), "Idle -- last run: %lu / %lu (%s)",
                      (unsigned long)s_current_code, (unsigned long)s_total_codes,
                      proto.name);
    }
    lv_label_set_text(s_status_label, buf);

    if (s_progress_bar && s_total_codes > 0) {
        int32_t pct = (int32_t)((uint64_t)s_current_code * 100 / s_total_codes);
        lv_bar_set_value(s_progress_bar, pct, LV_ANIM_OFF);
    }
}

void finish_run(EndReason reason) {
    s_active = false;
    // Important fix (Task 8 review round): update_ui() must run BEFORE the
    // terminal message is written below, not after. update_ui() has its own
    // unconditional status-label write (the "Running .../ Idle -- ..."
    // text) -- calling it AFTER composing the terminal message clobbered
    // that message immediately in the same call, so e.g. the transmit-
    // failure reason below never actually reached the screen (only the
    // serial log). update_ui() still does the real work this function
    // needs beyond the label: flips the protocol dropdown back to enabled
    // and the toggle button back to "Start", and (harmlessly) repaints the
    // progress bar/status label with the generic idle text that gets
    // overwritten immediately after.
    update_ui();
    if (!s_status_label) return;

    const BruteProtocol &proto = kBruteProtocols[s_protocol_idx];
    char buf[112];
    switch (reason) {
        case EndReason::kCompleted:
            std::snprintf(buf, sizeof(buf), "Done -- swept all %lu codes (%s)",
                          (unsigned long)s_total_codes, proto.name);
            break;
        case EndReason::kUserStopped:
            std::snprintf(buf, sizeof(buf), "Stopped by user at %lu / %lu (%s)",
                          (unsigned long)s_current_code, (unsigned long)s_total_codes,
                          proto.name);
            break;
        case EndReason::kTransmitFailed:
            std::snprintf(buf, sizeof(buf), "Stopped -- transmit failed at code %lu (%s)",
                          (unsigned long)s_current_code, Rf433Replay::failure_reason());
            break;
    }
    lv_label_set_text(s_status_label, buf);
}

void toggle_click_cb(lv_event_t *) {
    if (s_active) {
        Serial.println("quarky-tab5: [rf433-bruteforce] Stop tapped by user");
        finish_run(EndReason::kUserStopped);
        return;
    }

    if (s_protocol_dropdown) {
        s_protocol_idx = (int)lv_dropdown_get_selected(s_protocol_dropdown);
    }
    const BruteProtocol &proto = kBruteProtocols[s_protocol_idx];
    s_current_code = 0;
    s_total_codes = 1u << proto.bits;
    s_active = true;
    Serial.printf("quarky-tab5: [rf433-bruteforce] Start tapped -- protocol=%s, "
                  "%lu candidate codes\n", proto.name, (unsigned long)s_total_codes);
    update_ui();
}

lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("RF433 Bruteforce", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *warn = lv_label_create(content);
    lv_label_set_text(warn, "Transmits every candidate code for the selected "
                             "protocol. Point RF433T at the target device.");
    lv_label_set_long_mode(warn, LV_LABEL_LONG_WRAP);

    s_protocol_dropdown = lv_dropdown_create(content);
    {
        char options[256];
        options[0] = '\0';
        for (int i = 0; i < kBruteProtocolCount; i++) {
            if (i > 0) std::strcat(options, "\n");
            std::strcat(options, kBruteProtocols[i].name);
        }
        lv_dropdown_set_options(s_protocol_dropdown, options); // copies into LVGL's own storage
        lv_dropdown_set_selected(s_protocol_dropdown, s_protocol_idx);
    }

    s_status_label = lv_label_create(content);
    lv_label_set_long_mode(s_status_label, LV_LABEL_LONG_WRAP);

    s_progress_bar = lv_bar_create(content);
    lv_obj_set_size(s_progress_bar, LV_PCT(100), 16);
    lv_bar_set_range(s_progress_bar, 0, 100);
    lv_bar_set_value(s_progress_bar, 0, LV_ANIM_OFF);

    s_toggle_btn = lv_button_create(content);
    s_toggle_label = lv_label_create(s_toggle_btn);
    lv_obj_add_event_cb(s_toggle_btn, toggle_click_cb, LV_EVENT_CLICKED, nullptr);

    update_ui(); // paints initial "Idle -- <default protocol> selected" state

    // Teardown on delete (Back button, same convention as rf433_scan.cpp's
    // build_screen()). Null widget pointers FIRST -- update_ui()/finish_run()
    // already no-op on null widgets, so this removes any ordering dependency
    // on LVGL's event-callback sequencing.
    //
    // Important fix (Task 8 review round): a run in progress IS stopped here
    // now, mirroring rf433_scan.cpp's build_screen() teardown, which calls
    // set_capture_active(false) when was_active on its own screen's DELETE
    // event. The original "leave it running, like Rf433Replay's background
    // transmit task" reasoning does not actually transfer from that
    // precedent: Rf433Replay's in-flight task is a single, sub-second,
    // uncancellable burst already committed to hardware by the time the
    // screen could close. This sweep is a cancellable, potentially
    // multi-minute operation (a full 12-bit keyspace at ~50ms/code is
    // ~3.4 minutes) -- leaving it running silently after the user backs out
    // is a real UX/safety issue (the Tab5 keeps transmitting with no visible
    // indication), and it interacts badly with the rest of this subsystem:
    // if something else claims the GPIO53 arbiter mid-sweep (e.g. opening
    // the NFC screen), the sweep would silently self-terminate via
    // transmit()'s refusal path with no UI open to report it. Setting
    // s_active = false directly (not via finish_run(), which would touch
    // already-nulled widgets pointlessly) is sufficient: poll() no-ops the
    // instant s_active is false, so no in-flight candidate is left queued.
    lv_obj_add_event_cb(content, [](lv_event_t *) {
        bool was_active = s_active;
        s_status_label = nullptr;
        s_progress_bar = nullptr;
        s_protocol_dropdown = nullptr;
        s_toggle_btn = nullptr;
        s_toggle_label = nullptr;
        if (was_active) {
            Serial.println("quarky-tab5: [rf433-bruteforce] Screen closed mid-run -- "
                            "stopping sweep");
            s_active = false;
        }
    }, LV_EVENT_DELETE, nullptr);

    return screen;
}

void start() { ScreenStack::push(build_screen()); }

} // namespace

void register_module() {
    g_registry.register_module({"rf433_bruteforce", "RF433 Bruteforce",
                                 Category::RF433, Affinity::TAB5_NATIVE,
                                 start, nullptr});
}

void poll() {
    if (!s_active) return;
    if (Rf433Replay::is_busy()) return; // previous candidate still transmitting -- wait

    if (s_current_code >= s_total_codes) {
        Serial.println("quarky-tab5: [rf433-bruteforce] keyspace exhausted -- run complete");
        finish_run(EndReason::kCompleted);
        return;
    }

    build_candidate(s_protocol_idx, s_current_code, s_candidate);
    Rf433Replay::transmit(s_candidate);

    if (!Rf433Replay::is_busy()) {
        // transmit() refused outright -- see Rf433Replay::transmit()'s doc
        // comment (rf433_replay.h) for every refusal reason (a replay/RX
        // capture already in flight, no edges, the GPIO53 arbiter held by
        // an NFC/RFID2 session, or xTaskCreatePinnedToCore() OOM). None of
        // these clear themselves by blindly retrying the SAME code next
        // tick, so stop the run rather than spinning forever;
        // Rf433Replay::failure_reason() is surfaced via finish_run().
        Serial.printf("quarky-tab5: [rf433-bruteforce] transmit() REFUSED at code "
                      "%lu -- stopping run (%s)\n", (unsigned long)s_current_code,
                      Rf433Replay::failure_reason());
        finish_run(EndReason::kTransmitFailed);
        return;
    }

    s_current_code++;
    update_ui();
}

} // namespace Rf433Bruteforce
