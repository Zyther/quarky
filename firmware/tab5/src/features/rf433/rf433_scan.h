#pragma once

#include "rf433_common.h"
#include <cstddef>
#include <cstdint>

namespace Rf433Scan {

// Maximum number of edge samples stored per captured signal burst
constexpr size_t kMaxEdgesPerSignal = 512;

// Maximum number of signals retained in memory for the scan list
constexpr size_t kMaxCapturedSignals = 16;

struct CapturedSignal {
    Rf433Common::EdgeSample edges[kMaxEdgesPerSignal];
    size_t edge_count;
    uint32_t captured_at_ms;

    // Monotonically increasing, never reused within a session. Identifies
    // this capture independent of its slot in the (ring-buffered) session
    // array, so a UI element created for this signal stays correct even
    // after later captures shift it out of its original array index.
    uint32_t capture_id;

    // True if the source burst hit kMaxEdgesPerSignal before its actual
    // end-of-burst gap arrived -- edges beyond the cap were dropped and this
    // signal's edges[]/edge_count do not represent the whole burst.
    bool truncated;
};

// Register the RF433 Scan feature module with the registry
void register_module();

// Feature module entry point (builds and pushes the screen)
void start();

// Called from main.cpp's loop() to drain the ISR ring buffer and update UI
void poll();

// Number of signals currently in the session buffer
size_t signal_count();

// Access a captured signal by index (returns nullptr if out of bounds)
const CapturedSignal *get_signal(size_t index);

} // namespace Rf433Scan
