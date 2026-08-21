#pragma once

#include "../hal/istorage.h"

// ===========================================================================
// Generic SD file browser (Phase 3 Task 22): a reusable "list files in a
// directory via IStorage, let the user tap one, hand back the full path"
// component, generalized from wifi_evil_portal.cpp's template picker
// (features/wifi/wifi_evil_portal.cpp -- its `s_template_dropdown`, scanned
// via `storage.list_files(kPortalDir, ".html", ...)`) so RF433 (and, once
// Task 18/ir_clone.cpp exists, IR) can call into ONE shared picker instead of
// each feature growing its own similar-but-different one.
//
// SCOPE, disclosed honestly rather than overclaimed (task-22-controller-notes.md):
// this is a FLAT, single-directory browser, not a recursive directory
// navigator. `IStorage::list_files(dir, ext_filter, names_out, max_names)`
// (hal/istorage.h) lists basenames matching an extension filter in exactly
// one directory -- it does not recurse, and its real, concrete implementation
// (StorageSD::list_files(), hal/storage_sd.cpp) explicitly SKIPS directory
// entries (`if (!entry.isDirectory())`) with no parallel primitive exposed
// through IStorage to enumerate a directory's OWN subdirectories. Adding
// "navigate into a subdirectory" on top of that would mean inventing a new
// storage primitive beyond what Tasks 1-21 established -- out of scope for
// this task per the controller notes' own ruling. This component instead
// generalizes wifi_evil_portal.cpp's actual real scope (list one directory,
// filtered by extension, let the user pick) by taking the directory and
// extension as parameters instead of hardcoding them.
//
// Single-instance component (module-level static state, same shape as
// wifi_evil_portal.cpp's own module statics) -- only one browser screen is
// ever open at a time, matching how it's actually used (pushed from a
// feature screen's button tap, popped on selection or Back).
// ===========================================================================

namespace FileBrowser {

// Called with the full "dir/name" path of the file the user tapped.
// user_data is passed through unchanged from push()'s own argument.
using SelectCallback = void (*)(const char *path, void *user_data);

// Pushes (via ScreenStack::push) a sub-screen titled `title` listing every
// file in `dir` whose name ends with `ext_filter` (e.g. ".sub"), read via
// `storage.list_files()`. Tapping a row calls `on_select(full_path,
// user_data)` and then pops the browser screen (ScreenStack::pop()) --
// `full_path` is a stack/local buffer valid only for the duration of that
// callback, so callers that need the path afterward must copy it. Tapping
// this screen's own Back button (screen_scaffold's standard chrome) just
// pops without calling on_select.
//
// `max_entries` is clamped to this component's own fixed internal capacity
// (see file_browser.cpp) -- callers do not need to size any buffer
// themselves. If `dir` has no matching files (or doesn't exist / SD isn't
// mounted -- list_files() returns 0 for all of these, per its own documented
// contract), the screen shows a "no files found" placeholder instead of an
// empty list.
void push(IStorage &storage, const char *title, const char *dir,
          const char *ext_filter, SelectCallback on_select,
          void *user_data = nullptr, int max_entries = 32);

} // namespace FileBrowser
