// UI half of NfcTagLibrary (register_module() + the browse screen) --
// deliberately split from nfc_tag_library.cpp (which holds save()/list()/
// load()) because this half depends on LVGL/FeatureRegistry/Arduino, none of
// which are available to `pio test -e native` (platformio.ini's [env:native]
// has no LVGL lib_dep at all). Mirrors this project's existing convention of
// keeping a pure format/storage module (rf433_sub_format.cpp, Task 21)
// separate from its feature's own UI module (rf433_scan.cpp). Both this file
// and nfc_tag_library.cpp implement the single API declared in
// nfc_tag_library.h -- callers (main.cpp) see one namespace, unaware of the
// physical split.
#include "nfc_tag_library.h"
#include "nfc_emulate.h" // Task 24: "Emulate" action on a saved tag's detail view

#include "../../hal/storage_sd.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"

#include <feature_registry.h>
#include <lvgl.h>

#include <cstdio>
#include <cstring>

extern FeatureRegistry g_registry;
extern StorageSD storage; // defined in main.cpp (Phase 1 Task 10); handed to
                          // NfcTagLibrary's IStorage&-taking functions below,
                          // same extern wifi_evil_portal.cpp declares for
                          // this exact real-hardware storage instance.

namespace NfcTagLibrary {

// --- Browse screen ----------------------------------------------------------
//
// List + tap-to-view (plan Step 4). NOTE: nothing registered by this module
// currently calls save() from a live scan -- this task's own file list
// (nfc_tag_library.h/.cpp, its test, main.cpp) doesn't include
// nfc_read.cpp, so no "Save to Library" button exists yet on that screen's
// found-tag result. That wiring (a small hook in nfc_read.cpp's kFound state
// calling NfcTagLibrary::save()) is left for a follow-up, and is called out
// explicitly in this task's report as a concern: the plan's own PAUSE FOR
// HARDWARE step ("scan-then-save") has nothing to press without it.

namespace {

constexpr int kMaxEntries = 32; // generous for a browse list; bounded, not
                                // unbounded RAM use, same reasoning as
                                // wifi_evil_portal.cpp's kMaxTemplates.

lv_obj_t *s_list = nullptr;
lv_obj_t *s_detail_label = nullptr;
lv_obj_t *s_emulate_btn = nullptr;
lv_obj_t *s_remove_btn = nullptr;
char s_names[kMaxEntries][64];
int s_entry_count = 0;

// The tag currently shown in the detail view (valid only while
// s_has_current_tag is true), so the "Emulate" button below can act on
// whatever was last tapped without re-reading from SD. Mirrors nfc_read.cpp's
// s_last_found_tag/kFound-state pattern for the same reason: the click
// handler that uses this runs on a LATER event than the one that set it.
// s_current_name is the same idea for "Remove" (2026-08-24, project owner's
// own explicit request): NfcTagLibrary::remove() takes a filename, not a
// TagInfo, so this is tracked alongside s_current_tag rather than derived
// from it.
NfcCommon::TagInfo s_current_tag{};
char s_current_name[64] = "";
bool s_has_current_tag = false;

void clear_detail() {
    if (s_detail_label != nullptr) {
        lv_label_set_text(s_detail_label, "Tap a saved tag to view its details.");
    }
    s_current_name[0] = '\0';
    s_has_current_tag = false;
}

void show_tag(int idx) {
    if (idx < 0 || idx >= kMaxEntries) {
        return;
    }
    NfcCommon::TagInfo tag{};
    if (!load(storage, s_names[idx], &tag)) {
        if (s_detail_label != nullptr) {
            lv_label_set_text(s_detail_label, "Failed to load tag record.");
        }
        s_has_current_tag = false;
        return;
    }
    char uid_str[64];
    NfcCommon::format_uid(tag.uid, tag.uid_len, uid_str, sizeof(uid_str));
    char buf[224];
    // page_count is surfaced because it is exactly what decides whether the
    // "Emulate" button below can answer a reader's READ commands or only its
    // anticollision -- see nfc_emulate.h's own scope comment.
    std::snprintf(buf, sizeof(buf),
                  "%s\n%s\nUID (%u bytes): %s\nContent: %s",
                  s_names[idx], tag.type_name, (unsigned)tag.uid_len, uid_str,
                  (tag.page_count > 0) ? "captured pages -- full emulation"
                                       : "none -- UID/SAK/ATQA emulation only");
    if (tag.page_count > 0) {
        std::snprintf(buf + std::strlen(buf), sizeof(buf) - std::strlen(buf),
                      " (%u pages)", (unsigned)tag.page_count);
    }
    if (s_detail_label != nullptr) {
        lv_label_set_text(s_detail_label, buf);
    }
    s_current_tag = tag;
    std::strncpy(s_current_name, s_names[idx], sizeof(s_current_name) - 1);
    s_current_name[sizeof(s_current_name) - 1] = '\0';
    s_has_current_tag = true;
}

// Repopulates s_list from a fresh NfcTagLibrary::list() call. Used both for
// the screen's initial population and, as of the "Remove" action, to refresh
// in place after a deletion -- lv_obj_clean() (a real, standard LVGL
// function) removes s_list's existing button children without touching the
// screen itself, so this never has to pop/rebuild the whole screen or
// disturb ScreenStack.
void refresh_list() {
    lv_obj_clean(s_list);
    s_entry_count = list(storage, s_names, kMaxEntries);
    if (s_entry_count == 0) {
        lv_list_add_text(s_list, "No saved tags yet.");
        return;
    }
    for (int i = 0; i < s_entry_count; i++) {
        // Tapping a row loads and displays that saved tag's details --
        // index stashed as user_data, same pattern ble_clone.cpp's target
        // list and ble_gatt_explorer.cpp's characteristic list already
        // use for "row index -> action" click handlers.
        lv_obj_t *btn = lv_list_add_button(s_list, LV_SYMBOL_FILE, s_names[i]);
        lv_obj_add_event_cb(btn, [](lv_event_t *e) {
            int idx = (int)(intptr_t)lv_event_get_user_data(e);
            show_tag(idx);
        }, LV_EVENT_CLICKED, (void *)(intptr_t)i);
    }
}

lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("NFC Tag Library", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_list = lv_list_create(content);
    lv_obj_set_size(s_list, LV_PCT(100), LV_PCT(50));
    refresh_list();

    s_detail_label = lv_label_create(content);
    lv_label_set_text(s_detail_label, "Tap a saved tag to view its details.");

    // "Emulate" (Task 24), next to the existing load/view action: presents
    // the currently-shown saved tag's real UID/SAK/ATQA over Listen Mode.
    // NFC-unit only (see nfc_emulate.h's own header comment on why RFID2
    // cannot do this at all) -- but every tag in this library, regardless of
    // which unit originally scanned it, is emulated the same way, since
    // Listen Mode only needs the UID/SAK/ATQA already stored in TagInfo, not
    // which unit produced them.
    s_emulate_btn = lv_button_create(content);
    lv_obj_t *emulate_lbl = lv_label_create(s_emulate_btn);
    lv_label_set_text(emulate_lbl, "Emulate");
    lv_obj_add_event_cb(s_emulate_btn, [](lv_event_t *) {
        if (!s_has_current_tag) {
            if (s_detail_label != nullptr) {
                lv_label_set_text(s_detail_label, "Tap a saved tag first.");
            }
            return;
        }
        NfcEmulate::start(s_current_tag);
    }, LV_EVENT_CLICKED, nullptr);

    // "Remove" (2026-08-24, project owner's own explicit request): deletes
    // the currently-shown saved tag's .tag file from SD and refreshes the
    // list in place -- there was previously no way to remove a saved entry
    // at all, only save/view/emulate.
    s_remove_btn = lv_button_create(content);
    lv_obj_t *remove_lbl = lv_label_create(s_remove_btn);
    lv_label_set_text(remove_lbl, "Remove");
    lv_obj_add_event_cb(s_remove_btn, [](lv_event_t *) {
        if (!s_has_current_tag) {
            if (s_detail_label != nullptr) {
                lv_label_set_text(s_detail_label, "Tap a saved tag first.");
            }
            return;
        }
        // Copy the name out first: refresh_list() repopulates s_names, and
        // clear_detail() clears s_current_name -- neither should still be
        // read from after they run.
        char name[64];
        std::strncpy(name, s_current_name, sizeof(name) - 1);
        name[sizeof(name) - 1] = '\0';
        const bool ok = remove(storage, name);
        clear_detail();
        refresh_list();
        if (!ok && s_detail_label != nullptr) {
            lv_label_set_text(s_detail_label, "Failed to remove tag record.");
        }
    }, LV_EVENT_CLICKED, nullptr);

    lv_obj_add_event_cb(content, [](lv_event_t *) {
        s_list = nullptr;
        s_detail_label = nullptr;
        s_emulate_btn = nullptr;
        s_remove_btn = nullptr;
        s_has_current_tag = false;
    }, LV_EVENT_DELETE, nullptr);

    return screen;
}

void start() {
    ScreenStack::push(build_screen());
}

} // namespace

void register_module() {
    g_registry.register_module({"nfc_tag_library", "NFC: Tag Library",
                                Category::NFC, Affinity::TAB5_NATIVE,
                                start, nullptr});
}

} // namespace NfcTagLibrary
