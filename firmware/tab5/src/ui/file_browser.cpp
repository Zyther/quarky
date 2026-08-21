#include "file_browser.h"
#include "screen_scaffold.h"
#include "screen_stack.h"
#include <lvgl.h>
#include <cstdio>
#include <cstring>

namespace FileBrowser {

namespace {

// Fixed internal capacity -- mirrors wifi_evil_portal.cpp's kMaxTemplates(8)
// idiom of a small, generous-for-real-use compile-time cap rather than a
// dynamic list. 32 comfortably covers a directory of SD-copied capture/
// interop files without the unbounded-scan risk list_files() itself already
// guards against internally (StorageSD::list_files()'s kMaxEntriesScanned).
constexpr int kMaxEntries = 32;

char s_dir[96];
char s_names[kMaxEntries][64];
int s_count = 0;
SelectCallback s_on_select = nullptr;
void *s_user_data = nullptr;

void row_click_cb(lv_event_t *e) {
    int idx = static_cast<int>(reinterpret_cast<intptr_t>(lv_event_get_user_data(e)));
    if (idx < 0 || idx >= s_count) return; // defensive; row buttons are only
                                            // ever created for 0..s_count-1

    char path[160];
    std::snprintf(path, sizeof(path), "%s/%s", s_dir, s_names[idx]);

    // Snapshot the callback/user_data into locals before popping -- pop()
    // deletes this screen (LV_EVENT_DELETE fires on `content`, though this
    // component adds no handler for it since it holds no lv_obj_t* statics
    // that outlive the screen), and s_on_select/s_user_data are reset by the
    // NEXT push() call, not by pop() itself, so reading them after pop()
    // would still be technically safe here -- but capturing first keeps this
    // correct even if a future change makes pop() or on_select re-entrant
    // (e.g. on_select itself calling FileBrowser::push() again for a
    // different picker).
    SelectCallback cb = s_on_select;
    void *user_data = s_user_data;
    ScreenStack::pop();
    if (cb) cb(path, user_data);
}

} // namespace

void push(IStorage &storage, const char *title, const char *dir,
          const char *ext_filter, SelectCallback on_select, void *user_data,
          int max_entries) {
    std::strncpy(s_dir, dir, sizeof(s_dir) - 1);
    s_dir[sizeof(s_dir) - 1] = '\0';
    s_on_select = on_select;
    s_user_data = user_data;

    int cap = max_entries;
    if (cap > kMaxEntries) cap = kMaxEntries;
    if (cap < 0) cap = 0;
    s_count = storage.list_files(dir, ext_filter, s_names, cap);

    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen(title, &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *list = lv_list_create(content);
    lv_obj_set_size(list, LV_PCT(100), LV_PCT(100));

    if (s_count == 0) {
        char msg[128];
        std::snprintf(msg, sizeof(msg), "No %s files found in %s", ext_filter, dir);
        lv_list_add_text(list, msg);
    } else {
        for (int i = 0; i < s_count; i++) {
            lv_obj_t *btn = lv_list_add_button(list, LV_SYMBOL_FILE, s_names[i]);
            lv_obj_add_event_cb(btn, row_click_cb, LV_EVENT_CLICKED,
                                 reinterpret_cast<void *>(static_cast<intptr_t>(i)));
        }
    }

    ScreenStack::push(screen);
}

} // namespace FileBrowser
