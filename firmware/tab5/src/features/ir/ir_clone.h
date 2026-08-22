#pragma once

// ===========================================================================
// IR Clone -- universal remote / multi-profile clone (Phase 3 Task 18),
// against the real community Flipper-IRDB copy loaded onto the Tab5's SD
// card (`/quarky/ir/flipperdb/...`).
//
// REAL, ON-DISK STRUCTURE THIS NAVIGATES: confirmed by the controller
// inspecting the real hardware directly this session (see the plan doc's
// Task 18 "CORRECTED/EXPANDED 2026-08-22" section) -- deep and
// INCONSISTENT in depth (e.g. `flipperdb/Window_cleaners/HOBOT/*.ir`, 2
// levels under the browsable root; `flipperdb/_Converted_/IR_Plus/R/
// REVOX/*.ir`, 4 levels). The existing generic file browser (Task 22,
// ui/file_browser.h) is explicitly flat/single-directory only and cannot
// navigate this -- this module is a genuine recursive folder-by-folder
// navigator instead, built as its own screen rather than extending
// FileBrowser in place (its existing callers -- rf433_scan.cpp's .sub
// replay picker -- stay untouched).
//
// NAVIGATION DESIGN: one screen per directory level, each a fresh
// ScreenStack::push() of a new build_dir_screen(dir) instance (not a
// single mutable screen reused in place). Descending into a subfolder
// pushes a new instance for that subfolder; the standard Back button every
// screen already gets from ui/screen_scaffold.h's build_sub_screen()
// (wired to ScreenStack::pop(), see screen_stack.cpp) is therefore already
// exactly the "ascend one level" affordance this task needs -- no separate
// Up control was built. Each level lists ONLY its own immediate
// subdirectories (IStorage::list_dirs(), added this task) and `.ir` files
// (IStorage::list_files(), pre-existing) -- never a whole-tree walk, which
// is what caused a real task-watchdog reset earlier this session (see the
// plan doc). Real observed max depth (root + flipperdb + up to 4 more
// levels + a file's own signal screen) fits within ScreenStack::kMaxDepth
// (8) but is close to it -- if the real corpus turns out to have a branch
// even one level deeper than the two examples confirmed this session,
// ScreenStack::push() will silently stop tracking depth (see its own
// implementation) rather than crash; flagged here as a real, disclosed,
// not-yet-hardware-tested edge case.
//
// Tapping a `.ir` file reads + IrFileFormat::read()s it (may contain
// multiple named signals, per the real HOBOT samples' own ~11-12 buttons
// each) and shows ONE BUTTON PER PARSED SIGNAL, each labeled with that
// signal's real `name` field -- directly satisfying the project owner's
// own explicit requirement from earlier this session. Tapping a signal
// button transmits it: `kRaw` signals go straight to
// IrCommon::transmit_raw(); `kParsed` signals are encoded first via
// ir_nec_encode.h (NEC/NECext only -- any other real protocol name is
// refused with a clear on-screen message, not silently ignored or
// crashed on).
//
// No poll(): sending one button's signal is a single blocking
// IrCommon::transmit_raw() call (well under 100ms), the same risk profile
// as ir_tvbgone.cpp's own per-code transmit call -- only iterating MANY
// signals in one loop would need poll()-driven chunking, and this module
// never does that (one tap sends exactly one signal).
// ===========================================================================

namespace IrClone {

// Registers this module's launcher tile (Category::IR, Affinity::
// TAB5_NATIVE). Call once from setup(), before Shell::build().
void register_module();

} // namespace IrClone
