#pragma once

#include <cstddef>
#include <cstdint>

// ===========================================================================
// CC1101 scan/capture + protocol decode + hot/cold signal finder
// (Phase 10 Task 3).
//
// Receive-side mirror of firmware/tab5/src/features/rf433/{rf433_common,
// rf433_scan}.h: an attachInterrupt(CHANGE)-driven edge-timing capture on
// Cc1101Hw::gdo0_pin() (the CC1101's GDO0 line, put into async/direct RX mode
// via Cc1101Hw::enable_async_rx() -- see that function's own doc comment for
// why this is the same real hardware feature Poseidon's cc1101_hw.cpp
// (ELECHOUSE_cc1101.SetRx() + pinMode(CC1101_GDO0, INPUT)) and Bruce's
// SmartRC-based rf_*.cpp modules both use under a different driver's naming),
// ring-buffered and drained from poll() -- same portMUX-protected ISR ring
// shape rf433_common.cpp already established and real-hardware-hardened
// (runaway-interrupt self-disarm ceiling, oldest-first drain semantics).
// Ported here as the SAME pattern (not a new design), scoped down: CC1101's
// M-Bus pins are dedicated (no shared-with-I2C GPIO53 arbiter concern RF433
// has), so there is no equivalent of Gpio53Arbiter here.
//
// Donor reference (per this phase's spec Section 2 / Phase 5 spec Section 1):
// Poseidon subghz_scan.cpp (real capture_now()/gdo0_isr() edge-timing
// technique, MIN_PULSE_US/GAP_TIMEOUT_US-style burst delimiting) and
// subghz_jam_detect.cpp (real baseline-mean + trigger-delta RSSI monitoring,
// reused below as the "hot/cold" continuous-RSSI mode -- Poseidon's own
// jam-detection alarm logic and this phase's hot/cold signal-finder are the
// same underlying technique, continuous RSSI sampling against a learned
// floor, just surfaced as a different UI).
//
// Protocol decode: shared/subghz_proto's SubghzProto::decode() (Task 2).
// RAW is first-class (project owner, 2026-08-25, see the plan's Task 3
// Context note): a burst that decode() does not recognize is still a fully
// valid, save/replayable CapturedSignal -- see cc1101_record.h.
// ===========================================================================

namespace Cc1101Scan {

struct EdgeSample {
    uint32_t timestamp_us;
    bool level; // GPIO level AFTER the edge -- same semantics as
                // Rf433Common::EdgeSample / SubghzProto::EdgeSample.
};

// Same real-hardware-derived sizing as Rf433Scan::kMaxEdgesPerSignal
// (rf433_scan.h) -- same class of signal (OOK burst from a fixed-code or
// rolling-code remote), same PSRAM-headroom reasoning (kMaxCapturedSignals *
// sizeof(CapturedSignal) is heap-allocated in PSRAM below, not a static
// array, for the identical internal-DRAM-exhaustion reason rf433_scan.cpp's
// s_signals allocation comment documents in full).
constexpr size_t kMaxEdgesPerSignal = 8192;
constexpr size_t kMaxCapturedSignals = 16;

struct CapturedSignal {
    EdgeSample edges[kMaxEdgesPerSignal];
    size_t edge_count = 0;
    uint32_t captured_at_ms = 0;
    uint32_t capture_id = 0;   // stable, 1-based, never reused this session
    bool truncated = false;    // edges[] hit kMaxEdgesPerSignal before the
                                // real burst ended
    uint32_t freq_hz = 433920000u; // real tuned frequency AT CAPTURE TIME --
                                    // CC1101 is variable-frequency (unlike
                                    // RF433's fixed 433.92MHz), so this must
                                    // travel with the signal for .sub
                                    // encode/replay to use the right value.
};

void register_module();
void start();

// Called from main.cpp's loop(). No-ops unless the scan screen is open or a
// capture/hot-cold session is active.
void poll();

size_t signal_count();
const CapturedSignal *get_signal(size_t index);

// True while an edge capture (not hot/cold RSSI mode) is active -- consulted
// by Cc1101Replay so TX can refuse while this module's ISR is armed on the
// same GDO0 pin (mirrors Rf433Common::is_capturing() / Rf433Replay's RX/TX
// exclusion check, same real hazard: one physical pin, two directions).
bool is_capturing();

} // namespace Cc1101Scan
