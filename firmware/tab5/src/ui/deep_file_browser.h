#pragma once

#include "../hal/istorage.h"

// ===========================================================================
// Recursive SD directory browser (Phase 10 Task 4 follow-up, 2026-08-25) --
// generalizes IrClone's own real, working Flipper-IRDB navigator
// (features/ir/ir_clone.cpp's build_dir_screen()/on_subdir_click()) into a
// reusable component, the same way file_browser.h generalized
// wifi_evil_portal.cpp's template picker (Task 22). Unlike FileBrowser
// (Task 22), which is explicitly flat/single-directory only (see that
// header's own scope comment), this walks real subdirectories via
// IStorage::list_dirs() -- needed because the real on-SD Flipper SubGhz
// database (`/quarky/sub/flipperdb/...`) is genuinely deep, the same way
// the Flipper-IRDB is.
//
// Real difference from IrClone's own browser: IR files can hold multiple
// named signals, so IrClone shows a per-signal button grid on file tap.
// A `.sub` file is always exactly one capture -- there is nothing to pick
// once a file is tapped -- so this component calls on_select() directly on
// file tap (matching FileBrowser::push()'s own contract exactly) rather
// than opening a detail screen.
// ===========================================================================

namespace DeepFileBrowser {

using SelectCallback = void (*)(const char *path, void *user_data);

// Pushes a directory-browser screen rooted at `root_dir`, showing real
// subdirectories (tap to descend) and files matching `ext_filter` (tap to
// select). On file selection, calls on_select(full_path, user_data) then
// unwinds every screen this component pushed (however many directory
// levels deep the user navigated) back to whatever screen was on top when
// push() was called -- the caller never has to track navigation depth
// itself. Tapping any level's own Back button just pops one level, same as
// every other screen in this codebase.
void push(IStorage &storage, const char *root_dir, const char *ext_filter,
          SelectCallback on_select, void *user_data = nullptr);

} // namespace DeepFileBrowser
