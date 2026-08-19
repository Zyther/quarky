#include "nfc_common.h"

#include <cstdio>
#include <cstring>

namespace NfcCommon {

const char *format_uid(const uint8_t *uid,
                        uint8_t len,
                        char *out,
                        size_t out_len) {
    if (out == nullptr || out_len == 0) return "";
    if (uid == nullptr || len == 0) {
        out[0] = '\0';
        return out;
    }

    // Worst-case: "XX:" repeated (10 * 3 chars) + '\0'.
    // We'll build incrementally to avoid buffer overflow.
    size_t used = 0;
    bool truncated = false;
    for (uint8_t i = 0; i < len; i++) {
        const uint8_t b = uid[i];
        const bool last = (i + 1 == len);

        char tmp[8];
        if (last) {
            std::snprintf(tmp, sizeof(tmp), "%02X", b);
        } else {
            std::snprintf(tmp, sizeof(tmp), "%02X:", b);
        }

        const size_t need = std::strlen(tmp);
        if (used + need + 1 > out_len) {
            truncated = true;
            break;
        }
        std::memcpy(out + used, tmp, need);
        used += need;
        out[used] = '\0';
    }

    // A UID that did not fit used to end on the separator ("04:A3:"), which
    // reads as a complete value whose last byte happens to be missing. Replace
    // the dangling ':' with an ellipsis where there is room, and failing that
    // just drop it -- either way the string must not claim to be whole.
    if (truncated && used > 0 && out[used - 1] == ':') {
        if (used + 3 < out_len) {
            out[used++] = '.';
            out[used++] = '.';
            out[used++] = '.';
        } else {
            used--;
        }
        out[used] = '\0';
    }

    return out;
}

} // namespace NfcCommon

