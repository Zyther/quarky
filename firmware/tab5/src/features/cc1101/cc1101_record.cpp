#include "cc1101_record.h"
#include <subghz_sub_format.h>
#include <cstring>

namespace Cc1101Record {

const char kPresetName[] = "FuriHalSubGhzPresetOok270Async";

namespace {
// RAISED 2026-08-25 (real bug found on a real 209,356-byte Flipper SubGhz-DB
// capture, "Vehicles/Honda/Lock_Honda.sub", 53,249 real edges): this
// constant was left at 131072 BYTES after Cc1101Scan::kMaxEdgesPerSignal was
// separately raised to 131072 EDGES (a coincidental same-number, different-
// unit collision this comment used to encourage by citing the OLD 8192-edge
// cap without updating when that cap changed). 131072 bytes of RAW_Data text
// cannot hold anywhere near 131072 edges' worth of encoded durations (each
// duration is ~4-11 text bytes plus a separator) -- load() silently read only
// the file's first 131072 bytes (StorageSD::read_file()'s own documented
// behavior for a file larger than the buffer), truncating this real capture
// to 33,391 of its real 53,249 edges before SubghzProto::decode_sub() ever
// saw the rest. (out->truncated does get set correctly in that case -- this
// was a real silent-truncation bug, not a crash or a hard failure -- but a
// user loading a real, unremarkable-sized capture should not need the
// literal maximum edge count to trigger it.)
//
// Same real per-value sizing rationale as rf433_sub_format.cpp's own
// kMaxEncodedTextBytes (~13 bytes/value worst case + header + per-512-values
// line-prefix overhead), scaled by the SAME ratio rf433_sub_format.cpp
// itself used when it was sized for RF433's own 8192-edge cap (131072 bytes
// / 8192 edges = 16 bytes/edge) -- applied here to this module's real
// 131072-edge cap: 131072 * 16 = 2,097,152 bytes (2MB). Plain `new` on first
// call routes to PSRAM automatically on real hardware
// (CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096, already verified in this
// project) -- save() and load() each hold their own 2MB buffer (4MB total),
// trivial against this hardware's real 32MB PSRAM budget alongside
// cc1101_scan.cpp's own ~16MB of capture storage.
constexpr size_t kMaxEncodedTextBytes = 2097152;
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
