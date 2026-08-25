#include "rf433_protocol_decode.h"
#include "rf433_common.h"
#include "subghz_protocol_decode.h"
#include <cstring>

// ===========================================================================
// THIN WRAPPER over shared/subghz_proto's SubghzProto::decode() (Phase 10
// Task 2 extraction, 2026-08-25 -- see rf433_protocol_decode.h's own updated
// header comment for the full reasoning). The seven brand decoder state
// machines this file used to own directly (Holtek, Holtek HT12X, CAME, Nice
// FLO, Chamberlain, Ansonic, Linear -- originally ported from
// ~/src/unigeek-main/firmware/src/utils/rf/SubGhzDecoders.cpp, GPLv3) now
// live in shared/subghz_proto/src/subghz_protocol_decode.{h,cpp}, alongside
// an eighth (Princeton) added there for Phase 10. This file keeps only the
// RF433-specific parts shared/subghz_proto deliberately has no dependency
// on: the Rf433Scan::CapturedSignal/Rf433Common::EdgeSample[] ->
// duration[] conversion (build_durations(), below) and DecodedCode output
// shaping.
//
// This file's own public API (Rf433ProtocolDecode::decode(CapturedSignal,
// DecodedCode*)) is UNCHANGED -- test/test_rf433_protocol_decode.cpp passes
// against this wrapper with no modification, which is this extraction's own
// real acceptance gate (see the phase plan's Task 2).
// ===========================================================================

namespace Rf433ProtocolDecode {

namespace {

// Max pulse durations derivable from one CapturedSignal: one fewer than the
// edge sample cap, since a duration is the gap BETWEEN two consecutive edges
// (see build_durations() below).
constexpr size_t kMaxDurations = Rf433Scan::kMaxEdgesPerSignal - 1;

// EdgeSample{timestamp_us, level} -> alternating HIGH/LOW pulse durations.
//
// SubGhzDecoders::decode()'s real caller, CC1101Util::pollReceive()
// (CC1101Util.cpp:220-227), never faces this conversion at all: its capture
// front end is the ESP32 RMT peripheral (RmtRf::readFrame(), RmtRf.h:43-45),
// which is fed by hardware and produces signed pulse DURATIONS directly --
// there is no per-edge timestamp array to diff in the donor's own pipeline.
// This project's capture front end (rf433_common.h) is deliberately
// different -- a GPIO CHANGE interrupt timestamping edges with micros()
// (see that header's top-of-file comment for why: polling loop() was tried
// first and was orders of magnitude too slow for real OOK pulse widths) --
// so this conversion step is genuinely new work, not something to find a
// donor precedent for.
//
// Each duration is the gap between two consecutive edges: for i = 1 ..
// edge_count-1, duration[i-1] = edges[i].timestamp_us - edges[i-1].
// timestamp_us. That duration is the pulse at the level edges[i-1].level
// held until edges[i] (EdgeSample.level is documented as "the level this
// sample transitioned TO", rf433_common.h:30 -- so the level that HOLDS
// between edges[i-1] and edges[i] is edges[i-1].level, matching the donor
// decoders' "alternating HIGH/LOW durations" semantics exactly).
//
// There is no leading duration for edges[0]: it records only the level the
// pin transitioned TO, with no earlier timestamp in this CapturedSignal to
// measure how long the signal dwelled at the level before it. edges[0]
// ITSELF is still used -- it is duration[0]'s start point, not skipped --
// what is unavailable is the (unrecorded) pulse that preceded it, not a
// pulse this code fails to use. Inventing a value for that unrecorded pulse
// would be exactly the kind of fabricated timing value this project's "real
// sources only" discipline forbids. This costs at most one leading pulse's
// worth of decode context, which every decoder above tolerates (each self-
// syncs on its own header/preamble rather than requiring the capture's very
// first edge to be meaningful).
//
// MERGE FIX (round-2 review finding, checkable without hardware): the donor
// decoders assume level alternates strictly by array-index parity
// (sample_level() above) because the donor's own capture front end (the ESP32
// RMT peripheral) guarantees alternating levels by construction -- consecutive
// RMT items are always opposite polarity, so index parity alone is a safe
// stand-in for real level. This project's ISR-timestamped GPIO CHANGE capture
// carries NO such guarantee: real receiver-module squelch/glitch chatter can
// produce two or more CONSECUTIVE EdgeSamples with the SAME .level, which
// breaks the alternating-parity assumption every ported decoder above relies
// on. This is not theoretical -- the real fixture in
// test_rf433_protocol_decode.cpp has 143 of its 353 consecutive edge samples
// sharing the same .level as the sample immediately before them, in runs up
// to 26 samples long (almost certainly receiver chatter, not real signal).
// The old version of this function threw away EdgeSample.level entirely and
// handed the decoders one raw gap per adjacent edge pair, letting
// sample_level()'s parity guess silently mis-assign which gaps are "HIGH"
// and which are "LOW" every time a same-level run occurred.
//
// Fix: merge consecutive gaps that share the same held level (per
// EdgeSample.level -- the one piece of ground truth this project's capture
// actually has, which the donor's RMT-native pipeline never needed) into one
// true pulse, summing their durations, before handing the array to the
// decoders. The merged output is guaranteed to alternate level from one
// entry to the next by construction (a run of same-held-level gaps collapses
// to exactly one entry), which is what makes sample_level(i, phase)'s
// index-parity assumption valid again for the two-phase try loop below --
// decode() still doesn't know the capture's ABSOLUTE starting polarity, so
// it still tries both phases; this fix only restores the RELATIVE
// alternation between consecutive entries that the donor's hardware
// guaranteed and this project's does not.
size_t build_durations(const Rf433Scan::CapturedSignal &sig, unsigned int *out, size_t max_out) {
    if (sig.edge_count < 2) return 0;

    size_t out_count = 0;
    uint32_t merged_duration = 0;
    bool merged_level = false;
    bool have_pulse = false;

    for (size_t i = 1; i < sig.edge_count; i++) {
        uint32_t gap = static_cast<unsigned int>(sig.edges[i].timestamp_us - sig.edges[i - 1].timestamp_us);
        bool held_level = sig.edges[i - 1].level;

        if (!have_pulse) {
            merged_duration = gap;
            merged_level = held_level;
            have_pulse = true;
        } else if (held_level == merged_level) {
            // Same level as the run already in progress -- squelch/glitch
            // chatter, not a real transition. Fold into the pulse in
            // progress instead of handing decoders a spurious extra gap.
            merged_duration += gap;
        } else {
            if (out_count >= max_out) return out_count;
            out[out_count++] = merged_duration;
            merged_duration = gap;
            merged_level = held_level;
        }
    }
    if (have_pulse && out_count < max_out) {
        out[out_count++] = merged_duration;
    }
    return out_count;
}

} // namespace

bool decode(const Rf433Scan::CapturedSignal &sig, DecodedCode *out) {
    if (out == nullptr) return false;

    // Heap-allocated (plain `new`, lazy on first call) rather than a plain
    // static array -- UPDATED 2026-08-21: kMaxDurations now scales with
    // Rf433Scan::kMaxEdgesPerSignal (4096, was 512), making a plain static
    // here ~16KB of internal DRAM, the same exhaustion class
    // rf433_scan.cpp's s_signals comment documents. Plain `new` is portable
    // to the native host test target (plain heap there) and this project's
    // real, verified sdkconfig (CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096)
    // routes it to PSRAM automatically on the real target. Still not
    // reentrant/thread-safe (a single function-static buffer either way) --
    // this project calls decode() from a single task's poll loop today, so
    // that remains a constraint worth knowing, not a live bug.
    static unsigned int *dur = new unsigned int[kMaxDurations];
    size_t count = build_durations(sig, dur, kMaxDurations);
    // SubghzProto::decode()'s own guard, count < 8, mirrors
    // SubGhzDecoders::decode()'s own (SubGhzDecoders.cpp:1866) -- enforced
    // inside the shared library now, not duplicated here.
    SubghzProto::Match m{};
    if (!SubghzProto::decode(dur, static_cast<uint16_t>(count), &m)) return false;

    std::memset(out->protocol_name, 0, sizeof(out->protocol_name));
    std::strncpy(out->protocol_name, m.name, sizeof(out->protocol_name) - 1);
    out->code = m.key;
    out->bit_length = m.bits;
    return true;
}

} // namespace Rf433ProtocolDecode
