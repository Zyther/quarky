#pragma once

#include <cstddef>
#include <cstdint>

namespace NfcCommon {

struct TagInfo {
    uint8_t uid[10];
    uint8_t uid_len;
    char type_name[24];
};

// Formats uid as "04:A3:F1:..." (matching the project’s hex-with-colons style).
// Returns out for convenience (and never returns nullptr).
const char *format_uid(const uint8_t *uid,
                        uint8_t len,
                        char *out,
                        size_t out_len);

} // namespace NfcCommon

