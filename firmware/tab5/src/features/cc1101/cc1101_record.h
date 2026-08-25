#pragma once

#include "../../hal/istorage.h"
#include "cc1101_scan.h"

// ===========================================================================
// CC1101 record (.sub save) -- Phase 10 Task 4a.
//
// SD-backed wrapper around shared/subghz_proto's SubghzProto::encode_sub()
// (Task 2) -- same shape as rf433_sub_format.cpp's write(): pure in-memory
// encode, then one storage.write_capture_file() call. encode_sub() takes
// frequency/preset as parameters (Task 2's whole reason for existing
// alongside rf433_sub_format.cpp's fixed version) -- this module supplies
// CapturedSignal::freq_hz (the real tuned frequency AT CAPTURE TIME, not a
// fixed constant) and kPresetName below.
//
// RAW is first-class here too (project owner, 2026-08-25): save() has no
// dependency on SubghzProto::decode() having matched anything -- it always
// writes a real "Protocol: RAW" .sub file from the captured edges, matching
// cc1101_scan.h's own RAW-is-a-valid-result framing.
// ===========================================================================

namespace Cc1101Record {

// Real Flipper preset name for a CC1101-class variable-frequency OOK/ASK
// capture -- Poseidon's own subghz_scan.cpp save_sub() writes exactly this
// string (`f.println("Preset: FuriHalSubGhzPresetOok270Async")`,
// ~/src/poseidon-tab5/src/features/subghz_scan.cpp:112) for the same real
// hardware class (a CC1101 module, variable frequency), distinct from
// rf433_sub_format.cpp's fixed-433.92MHz-GPIO-bit-bang
// "FuriHalSubGhzPresetOok650Async". Written verbatim for real-Flipper-
// tooling compatibility, same "not acted on" caveat rf433_sub_format.h
// documents for its own kPresetName.
extern const char kPresetName[];

// Writes sig as a real Flipper ".sub" RAW file at `path` (any directory --
// callers land captures under /quarky/captures/subghz/ per this phase's
// spec Section 4.4). Returns false if SubghzProto::encode_sub() fails
// (see its own doc comment) or the SD write fails.
bool save(IStorage &storage, const char *path, const Cc1101Scan::CapturedSignal &sig);

// Loads a real Flipper ".sub" RAW file at `path` into *out (including its
// real Frequency: field, via SubghzProto::decode_sub() -- unlike RF433's
// fixed-frequency read(), the loaded frequency is whatever the file says, so
// Cc1101Replay::transmit() must retune before bit-banging it back out).
// Returns false if the file isn't a well-formed RAW .sub this module
// supports, or the SD read fails.
bool load(IStorage &storage, const char *path, Cc1101Scan::CapturedSignal *out);

} // namespace Cc1101Record
