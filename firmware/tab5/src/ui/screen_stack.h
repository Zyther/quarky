#pragma once
#include <lvgl.h>

class ScreenStack {
public:
    // Displays `screen` and pushes it onto the stack. If the stack is
    // already at kMaxDepth, refuses: `screen` is deleted (never displayed)
    // and the current screen stays as-is -- see kMaxDepth's own comment for
    // the real bug this refusal closes (a real, independent-code-review
    // finding, not a hypothetical).
    static void push(lv_obj_t *screen);
    static void pop();
    static void for_each(void (*fn)(lv_obj_t *));

private:
    // Real, reachable depth found via independent code review (Task 18,
    // ir_clone.cpp's Flipper-IRDB browser), 2026-08-22: the REAL, already-
    // observed 4-level-deep directory branch
    // (`flipperdb/_Converted_/IR_Plus/R/REVOX/*.ir`) produces a real
    // navigation chain of Shell root -> category screen -> IrClone's own
    // root -> flipperdb -> 4 more real subfolder levels -> a file's signal
    // screen = 9 levels -- one past the OLD value of 8. Before this fix,
    // push() called lv_screen_load() unconditionally even past kMaxDepth,
    // displaying a screen it silently failed to record in stack_[]/depth_
    // -- the next pop() would then delete the WRONG (previous) top screen
    // while leaking the actually-visible one and its own heap-allocated
    // state, and jump back two navigation levels instead of one. Raised to
    // 16 for real margin above the one real depth already found (a future
    // Flipper-IRDB branch even deeper than the one sampled this session is
    // plausible; 40+ other call sites across this codebase are nowhere
    // close to this many levels), AND push() itself now refuses cleanly
    // past the limit instead of silently doing this again for whatever
    // future caller eventually reaches it.
    static constexpr int kMaxDepth = 16;
    static lv_obj_t *stack_[kMaxDepth];
    static int depth_;
};
