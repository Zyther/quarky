#include "proto.h"

namespace c2proto {

int encode(const Frame &f, uint8_t *out, size_t out_cap) {
    if (f.payload_len > kMaxPayload) return -1;
    WireHeader hdr{};
    hdr.magic = kMagic;
    hdr.version = f.version;
    hdr.type = f.type;
    hdr.seq = f.seq;
    hdr.payload_len = f.payload_len;

    size_t total = sizeof(WireHeader) + f.payload_len;
    if (total > out_cap) return -1;

    memcpy(out, &hdr, sizeof(WireHeader));
    memcpy(out + sizeof(WireHeader), f.payload, f.payload_len);
    return (int)total;
}

bool decode(const uint8_t *in, size_t in_len, Frame &out) {
    if (in_len < sizeof(WireHeader)) return false;
    WireHeader hdr{};
    memcpy(&hdr, in, sizeof(WireHeader));
    if (hdr.magic != kMagic) return false;
    if (hdr.version != kVersion && hdr.version != 1) return false; // v1 is the only version today
    if (hdr.payload_len > kMaxPayload) return false;
    if (in_len < sizeof(WireHeader) + hdr.payload_len) return false;

    out.version = hdr.version;
    out.type = hdr.type;
    out.seq = hdr.seq;
    out.payload_len = hdr.payload_len;
    memcpy(out.payload, in + sizeof(WireHeader), hdr.payload_len);
    return true;
}

} // namespace c2proto
