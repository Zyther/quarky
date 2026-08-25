#include "cc1101_record.h"
#include <subghz_sub_format.h>
#include <cstring>

namespace Cc1101Record {

const char kPresetName[] = "FuriHalSubGhzPresetOok270Async";

namespace {
// Same sizing/allocation reasoning as rf433_sub_format.cpp's
// kMaxEncodedTextBytes/build_signed_durations() lazy `new` buffers: plain
// `new` on first call routes to PSRAM automatically on real hardware
// (CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096, already verified in this
// project) and is equally fine on the native host test target. Sized for
// Cc1101Scan::kMaxEdgesPerSignal (8192, same as RF433's own cap) worst case:
// ~13 bytes/value + header + per-512-values line overhead.
constexpr size_t kMaxEncodedTextBytes = 131072;
} // namespace

bool save(IStorage &storage, const char *path, const Cc1101Scan::CapturedSignal &sig) {
    static char *buf = new char[kMaxEncodedTextBytes];
    // SubghzProto::EdgeSample has identical layout/semantics to
    // Cc1101Scan::EdgeSample (both: uint32_t timestamp_us, bool level -- see
    // cc1101_scan.h's own comment on why this project keeps a plain,
    // dependency-free EdgeSample type per module rather than a single shared
    // one), so a reinterpret_cast avoids an edge-by-edge copy.
    static_assert(sizeof(SubghzProto::EdgeSample) == sizeof(Cc1101Scan::EdgeSample),
                  "SubghzProto::EdgeSample and Cc1101Scan::EdgeSample must stay layout-compatible");
    const auto *edges = reinterpret_cast<const SubghzProto::EdgeSample *>(sig.edges);
    size_t len = 0;
    if (!SubghzProto::encode_sub(sig.freq_hz, kPresetName, edges, sig.edge_count,
                                  buf, kMaxEncodedTextBytes, &len)) {
        return false;
    }
    return storage.write_capture_file(path, reinterpret_cast<const uint8_t *>(buf), len);
}

bool load(IStorage &storage, const char *path, Cc1101Scan::CapturedSignal *out) {
    static char *buf = new char[kMaxEncodedTextBytes];
    size_t len = 0;
    if (!storage.read_file(path, reinterpret_cast<uint8_t *>(buf), kMaxEncodedTextBytes, &len)) {
        return false;
    }
    auto *edges = reinterpret_cast<SubghzProto::EdgeSample *>(out->edges);
    size_t edge_count = 0;
    if (!SubghzProto::decode_sub(buf, len, &out->freq_hz, edges,
                                  Cc1101Scan::kMaxEdgesPerSignal, &edge_count)) {
        return false;
    }
    out->edge_count = edge_count;
    out->captured_at_ms = 0;
    out->capture_id = 0;
    // decode_sub() silently caps at edges_capacity with no truncation flag of
    // its own (see its doc comment) -- same signal read()'s own len ==
    // kMaxEncodedTextBytes check uses in rf433_sub_format.cpp: if the file
    // read hit the buffer cap, treat the load as possibly-truncated.
    out->truncated = (len == kMaxEncodedTextBytes) || (edge_count == Cc1101Scan::kMaxEdgesPerSignal);
    return true;
}

} // namespace Cc1101Record
