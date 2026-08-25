#include "deep_file_browser.h"

#include "screen_scaffold.h"
#include "screen_stack.h"

#include <lvgl.h>

#include <cstdio>
#include <cstring>

namespace DeepFileBrowser {
namespace {

// Same real bounds as ir_clone.cpp's own (2026-08-23 finding: the real
// on-SD Flipper-IRDB's root directory has more than 32 real entries --
// applies equally to the SubGhz database, same donor/bundling shape).
constexpr int kMaxDirEntries = 128;
constexpr int kMaxFileEntries = 128;
constexpr size_t kMaxPathLen = 160; // matches file_browser.h's own bound

IStorage *s_storage = nullptr;
char s_ext_filter[16] = "";
SelectCallback s_on_select = nullptr;
void *s_user_data = nullptr;

// How many screens THIS component has pushed and not yet popped -- lets
// on_file_click() unwind back to whatever screen was on top before push()
// was first called, regardless of how many directory levels deep the user
// navigated. Incremented in build_dir_screen() right before ScreenStack::
// push(); decremented on that same screen's own LV_EVENT_DELETE, so both a
// normal Back-button pop and this component's own programmatic unwind stay
// correctly tracked.
int s_push_count = 0;

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

void on_subdir_click(lv_event_t *e) {
    const char *path = static_cast<const char *>(lv_event_get_user_data(e));
    ScreenStack::push(build_dir_screen(path));
}

void on_file_click(lv_event_t *e) {
    const char *path = static_cast<const char *>(lv_event_get_user_data(e));
    SelectCallback cb = s_on_select;
    void *user_data = s_user_data;
    // Copy the path before unwinding: popping deletes the button (and its
    // user_data, this same `path` pointer) before cb() would otherwise run
    // if called after the pops.
    char path_copy[kMaxPathLen];
    std::strncpy(path_copy, path, sizeof(path_copy) - 1);
    path_copy[sizeof(path_copy) - 1] = '\0';

    int pops = s_push_count;
    for (int i = 0; i < pops; i++) ScreenStack::pop();

    if (cb != nullptr) cb(path_copy, user_data);
}

lv_obj_t *build_dir_screen(const char *dir) {
    const char *slash = std::strrchr(dir, '/');
    const char *leaf = (slash != nullptr && slash[1] != '\0') ? slash + 1 : dir;

    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen(leaf, &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    s_push_count++;
    lv_obj_add_event_cb(content, [](lv_event_t *) { s_push_count--; }, LV_EVENT_DELETE, nullptr);

    static char dir_names[kMaxDirEntries][64];
    static char file_names[kMaxFileEntries][64];
    bool dir_read_failed = false;
    bool file_read_failed = false;
    int dir_count = s_storage->list_dirs(dir, dir_names, kMaxDirEntries, &dir_read_failed);
    int file_count = s_storage->list_files(dir, s_ext_filter, file_names, kMaxFileEntries, &file_read_failed);

    lv_obj_t *list = lv_list_create(content);
    lv_obj_set_size(list, LV_PCT(100), LV_PCT(100));

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

} // namespace

void push(IStorage &storage, const char *root_dir, const char *ext_filter,
          SelectCallback on_select, void *user_data) {
    s_storage = &storage;
    std::strncpy(s_ext_filter, ext_filter, sizeof(s_ext_filter) - 1);
    s_ext_filter[sizeof(s_ext_filter) - 1] = '\0';
    s_on_select = on_select;
    s_user_data = user_data;
    s_push_count = 0;
    ScreenStack::push(build_dir_screen(root_dir));
}

} // namespace DeepFileBrowser
