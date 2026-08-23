#include "ir_clone.h"

#include "ir_common.h"
#include "ir_file_format.h"
#include "ir_nec_encode.h"
#include "../../hal/storage_sd.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"

#include <feature_registry.h>
#include <lvgl.h>

#include <cstdarg>
#include <cstdio>
#include <cstring>

extern FeatureRegistry g_registry;
extern StorageSD storage; // defined in main.cpp; same extern every other
                          // IStorage-consuming feature module declares for
                          // this real hardware instance (see
                          // nfc_tag_library_ui.cpp/ir_learn.cpp).

namespace IrClone {
namespace {

// Real, confirmed-this-session on-disk prefix for the bundled community
// database (`/quarky/ir/flipperdb/...`) -- the browsable root is one level
// up from that, `/quarky/ir`, so a future second bundle/category living
// alongside flipperdb/ (not just inside it) is browsable too without
// hardcoding the flipperdb name itself anywhere in this module.
constexpr char kRootDir[] = "/quarky/ir";

// Bounded per-directory-level listing capacity. Raised 32 -> 128
// (2026-08-23, real finding): the real on-SD-card Flipper-IRDB's own
// `flipperdb` root directory has MORE than 32 real category
// subdirectories (confirmed directly -- the project owner observed real
// folders past "Monitors" silently missing from the list) -- the old
// 32-entry cap, inherited from ui/file_browser.h's own flat-listing bound
// without re-deriving whether it was still appropriate for a real,
// wide (not just deep) community database, was truncating the root
// listing with no indication anything was cut off. 128 is a generous
// real bound above any observed real category count, not a spec limit;
// safe to raise this far now that the real memory constraint that would
// have made a larger static buffer risky is gone (see
// include/lv_conf.h's LV_MEM_SIZE fix, 2026-08-23 -- LVGL's own internal
// widget/style pool is now 1MB in PSRAM, not the 64KB static pool that
// made every real list row here costly).
//
// STILL NOT A TRUNCATION-SIGNALING FIX: if a real directory somehow has
// more than 128 entries, list_dirs()/list_files() still silently cap at
// this number with no on-screen indication, the same real gap this
// comment is itself flagging, just pushed to a much less likely real
// trigger. A full fix would need list_dirs()/list_files() to report
// "there were more than max_names real entries" back to the caller (a
// further IStorage interface change, out of scope for this fix).
constexpr int kMaxDirEntries = 128;
constexpr int kMaxFileEntries = 128;

// Real observed sample files have ~11-12 named signals each (both Hobot
// samples read in full this session); 32 is a generous bound well above
// that, not a spec limit -- distinct from IrFileFormat::kMaxSignalsPerFile
// (64, that module's own internal-processing cap), chosen smaller here
// purely to bound this screen's own heap allocation/button count.
constexpr size_t kMaxSignalsPerScreen = 32;

constexpr size_t kMaxPathLen = 160; // matches ui/file_browser.h's own
                                    // row_click_cb path[160] bound

// Established real default duty cycle for NEC-family transmission --
// same citation/reasoning as ir_tvbgone.cpp's decode_and_transmit() and
// ir_learn.cpp's kAssumedDutyCycle (this class of IR receiver is broadly
// duty-cycle-tolerant; not recoverable from a decoded protocol
// description any more than from a raw capture).
constexpr float kNecDutyCycle = 1.0f / 3.0f;

// Only one signal (file-detail) screen can ever be alive at a time --
// screens are strictly stacked/nested, and opening a second file always
// means popping back past the first one first -- so a single module-level
// status label pointer (reset to null on that screen's own teardown,
// same convention as every other feature screen in this codebase) is
// sufficient; no per-instance indirection needed.
lv_obj_t *s_status_label = nullptr;

void set_status(const char *msg) {
    if (s_status_label != nullptr) lv_label_set_text(s_status_label, msg);
}

void set_status_fmt(const char *fmt, ...) {
    if (s_status_label == nullptr) return;
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    lv_label_set_text(s_status_label, buf);
}

// Heap-duplicates a NUL-terminated string -- used to give each directory/
// file row button its OWN full-path copy as LVGL event user_data, freed on
// that same button's own LV_EVENT_DELETE. Necessary (rather than a single
// shared static path buffer, as e.g. ui/file_browser.h's single-instance
// s_dir/s_names use) because this screen is genuinely recursive: a parent
// directory's screen and a child directory's screen are BOTH alive at once
// (ScreenStack keeps the parent loaded-but-hidden underneath, see
// screen_stack.cpp's pop() only deleting the TOP screen) -- a shared
// static scratch buffer reused by the child's own build_dir_screen() call
// would corrupt the parent's still-live button data. Freed via `new[]`/
// `delete[]` rather than strdup()/free() to avoid depending on a
// non-standard-C++ libc extension's availability on this Arduino-ESP32
// target.
char *dup_str(const char *s) {
    size_t len = std::strlen(s);
    char *out = new char[len + 1];
    std::memcpy(out, s, len + 1);
    return out;
}

void free_user_data_str_cb(lv_event_t *e) {
    delete[] static_cast<char *>(lv_event_get_user_data(e));
}

lv_obj_t *build_dir_screen(const char *dir);
lv_obj_t *build_file_screen(const char *path);

void on_subdir_click(lv_event_t *e) {
    const char *path = static_cast<const char *>(lv_event_get_user_data(e));
    ScreenStack::push(build_dir_screen(path));
}

void on_file_click(lv_event_t *e) {
    const char *path = static_cast<const char *>(lv_event_get_user_data(e));
    ScreenStack::push(build_file_screen(path));
}

// Transmits one decoded signal, real-hardware transmit path shared by
// every signal button. Claims/releases IrCommon's RMT+GPIO53-arbiter
// ownership per tap (not held across taps) -- this screen has no ongoing
// state machine to keep it claimed between button presses, and releasing
// promptly after each single (well under 100ms) transmit lets NFC/RFID2/
// RF433 use PORT.A the rest of the time, same reasoning ir_learn.cpp's
// teardown_rx() releases as soon as a capture completes rather than
// holding the claim for the whole time the screen is open.
void transmit_signal(const IrFileFormat::IrSignal &sig) {
    if (!IrCommon::init()) {
        set_status("Transmit failed -- PORT.A is held by another owner "
                    "(NFC/RFID2 or RF433)");
        return;
    }

    bool ok = false;
    if (sig.type == IrFileFormat::SignalType::kRaw) {
        ok = IrCommon::transmit_raw(sig.data, sig.data_count, sig.frequency_hz, sig.duty_cycle);
        set_status_fmt(ok ? "Sent '%s' (raw)" : "Transmit failed: '%s' (raw)", sig.name);
    } else {
        uint16_t durations[IrNecEncode::kDurationsPerFrame];
        size_t count = 0;
        uint32_t carrier_hz = 0;
        if (!IrNecEncode::encode(sig, durations, IrNecEncode::kDurationsPerFrame, &count, &carrier_hz)) {
            // Real refusal, not a silent no-op or crash -- this project's
            // established convention (see ir_nec_encode.h's header).
            set_status_fmt("Protocol not supported: %s", sig.protocol);
            IrCommon::deinit();
            return;
        }
        ok = IrCommon::transmit_raw(durations, count, carrier_hz, kNecDutyCycle);
        set_status_fmt(ok ? "Sent '%s' (%s)" : "Transmit failed: '%s' (%s)", sig.name, sig.protocol);
    }
    IrCommon::deinit();
}

void on_signal_click(lv_event_t *e) {
    auto *sig = static_cast<const IrFileFormat::IrSignal *>(lv_event_get_user_data(e));
    transmit_signal(*sig);
}

// Frees the heap-allocated signal array a file screen decoded into --
// registered on that screen's own `content` object, so it's torn down
// exactly once, when the whole screen is deleted (ScreenStack::pop()).
// CORRECTED (Task 18 review round, 2026-08-22): this is safe NOT because
// parent and child teardown happen "together" -- verified against LVGL's
// own real lv_obj_tree.c: a parent's LV_EVENT_DELETE actually fires
// BEFORE its children are recursively deleted, the opposite of what this
// comment used to claim. It's safe because ScreenStack::pop() is a single
// synchronous, non-reentrant call on the main task: nothing between this
// callback freeing `signals` and the per-signal buttons' own subsequent
// deletion ever dereferences the now-dangling `&signals[i]` pointers those
// buttons hold as click-handler user_data -- button deletion itself never
// reads that user_data, only LV_EVENT_CLICKED does, and no click can fire
// once teardown has started. This reasoning would NOT hold under any
// future async/reentrant teardown path -- do not copy this pattern into
// one without re-deriving this analysis.
void free_signals_cb(lv_event_t *e) {
    delete[] static_cast<IrFileFormat::IrSignal *>(lv_event_get_user_data(e));
}

lv_obj_t *build_file_screen(const char *path) {
    const char *slash = std::strrchr(path, '/');
    const char *leaf = (slash != nullptr) ? slash + 1 : path;

    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen(leaf, &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    auto *signals = new IrFileFormat::IrSignal[kMaxSignalsPerScreen];
    lv_obj_add_event_cb(content, free_signals_cb, LV_EVENT_DELETE, signals);
    lv_obj_add_event_cb(content, [](lv_event_t *) { s_status_label = nullptr; }, LV_EVENT_DELETE, nullptr);

    bool truncated = false;
    size_t n = IrFileFormat::read(storage, path, signals, kMaxSignalsPerScreen, &truncated);

    if (n == 0) {
        lv_obj_t *lbl = lv_label_create(content);
        lv_label_set_text(lbl, "No parsable signals in this file.");
        lv_label_set_long_mode(lbl, LV_LABEL_LONG_WRAP);
        return screen;
    }

    if (truncated) {
        lv_obj_t *warn = lv_label_create(content);
        lv_label_set_text(warn, "This file had more signals than fit -- some are not shown.");
        lv_label_set_long_mode(warn, LV_LABEL_LONG_WRAP);
    }

    s_status_label = lv_label_create(content);
    lv_label_set_long_mode(s_status_label, LV_LABEL_LONG_WRAP);
    lv_label_set_text(s_status_label, "Tap a button below to transmit that command.");

    // One button per parsed signal -- directly satisfies the project
    // owner's own explicit requirement ("each command in an IR file is
    // represented by a button when the IR file is loaded").
    for (size_t i = 0; i < n; i++) {
        lv_obj_t *btn = lv_button_create(content);
        lv_obj_t *lbl = lv_label_create(btn);
        lv_label_set_text(lbl, signals[i].name);
        lv_obj_add_event_cb(btn, on_signal_click, LV_EVENT_CLICKED, &signals[i]);
    }

    return screen;
}

lv_obj_t *build_dir_screen(const char *dir) {
    const char *slash = std::strrchr(dir, '/');
    const char *leaf = (slash != nullptr && slash[1] != '\0') ? slash + 1 : dir;

    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen(leaf, &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    // Function-scope static scratch, not heap/stack per call -- safe to
    // reuse across recursive calls because each call fully consumes these
    // (copies into per-button heap-duplicated paths and LVGL's own
    // internally-copied label text) before returning; recursion into a
    // child directory's own build_dir_screen() call only ever happens
    // later, from a button's click handler, never nested inside this same
    // population loop. Same "reused scratch, safe because fully consumed
    // first" reasoning as ir_file_format.cpp's static `current` and
    // ui/file_browser.h's static s_names.
    static char dir_names[kMaxDirEntries][64];
    static char file_names[kMaxFileEntries][64];
    bool dir_read_failed = false;
    bool file_read_failed = false;
    int dir_count = storage.list_dirs(dir, dir_names, kMaxDirEntries, &dir_read_failed);
    int file_count = storage.list_files(dir, ".ir", file_names, kMaxFileEntries, &file_read_failed);

    lv_obj_t *list = lv_list_create(content);
    lv_obj_set_size(list, LV_PCT(100), LV_PCT(100));

    // Real distinction, not previously possible (see IStorage::list_dirs()/
    // list_files()'s own out_read_failed citation): a genuinely empty
    // directory and a directory whose SD read failed (most likely a busy
    // WiFi/BLE radio session starving the board's DMA-capable memory --
    // see the SDD ledger's Task 18 real-hardware investigation) used to
    // both silently show "Empty directory", making a real read failure
    // look identical to "there's really nothing here."
    if (dir_read_failed || file_read_failed) {
        lv_list_add_text(list, "Could not read this directory -- SD card read "
                               "failed (often caused by a busy WiFi/BLE radio "
                               "session leaving too little DMA-capable memory "
                               "free). Try again, or close other radio "
                               "features first.");
    } else if (dir_count == 0 && file_count == 0) {
        lv_list_add_text(list, "Empty directory.");
    }

    for (int i = 0; i < dir_count; i++) {
        char full[kMaxPathLen];
        std::snprintf(full, sizeof(full), "%s/%s", dir, dir_names[i]);
        lv_obj_t *btn = lv_list_add_button(list, LV_SYMBOL_DIRECTORY, dir_names[i]);
        char *path_copy = dup_str(full);
        lv_obj_add_event_cb(btn, on_subdir_click, LV_EVENT_CLICKED, path_copy);
        lv_obj_add_event_cb(btn, free_user_data_str_cb, LV_EVENT_DELETE, path_copy);
    }

    for (int i = 0; i < file_count; i++) {
        char full[kMaxPathLen];
        std::snprintf(full, sizeof(full), "%s/%s", dir, file_names[i]);
        lv_obj_t *btn = lv_list_add_button(list, LV_SYMBOL_FILE, file_names[i]);
        char *path_copy = dup_str(full);
        lv_obj_add_event_cb(btn, on_file_click, LV_EVENT_CLICKED, path_copy);
        lv_obj_add_event_cb(btn, free_user_data_str_cb, LV_EVENT_DELETE, path_copy);
    }

    return screen;
}

void start() { ScreenStack::push(build_dir_screen(kRootDir)); }

} // namespace

void register_module() {
    g_registry.register_module({"ir_clone", "IR Clone (Flipper-IRDB)", Category::IR,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

} // namespace IrClone
