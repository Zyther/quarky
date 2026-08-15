#include "ble_classify.h"
#include <cstring>

namespace BleClassify {

// Walks the AD structure list (length-byte + type-byte + payload,
// repeating) looking for a manufacturer-specific-data (0xFF) or
// service-data (0x16) AD structure, matching the same AD-parsing shape
// ble_hs_adv_parse_fields already does internally -- done manually here
// since we need the raw bytes for signature matching, not just the parsed
// device-name field ble_scan.cpp already extracts.
static const uint8_t *find_ad(const uint8_t *data, uint8_t len, uint8_t ad_type, uint8_t *out_len) {
    uint8_t i = 0;
    while (i + 1 < len) {
        uint8_t field_len = data[i];
        if (field_len == 0 || i + 1 + field_len > len) break;
        uint8_t type = data[i + 1];
        if (type == ad_type) {
            *out_len = field_len - 1;
            return &data[i + 2];
        }
        i += 1 + field_len;
    }
    return nullptr;
}

const char *classify(const uint8_t *adv_data, uint8_t adv_len) {
    uint8_t mfg_len = 0;
    const uint8_t *mfg = find_ad(adv_data, adv_len, 0xFF, &mfg_len);
    if (mfg != nullptr && mfg_len >= 4) {
        uint16_t company_id = mfg[0] | (mfg[1] << 8);
        if (company_id == 0x004C) { // Apple
            if (mfg_len >= 3 && mfg[2] == 0x02) return "iBeacon";
            if (mfg_len >= 3 && mfg[2] == 0x07) return "AirPods (Continuity)";
            if (mfg_len >= 3 && mfg[2] == 0x0F) return "Apple Nearby Action";
            if (mfg_len >= 3 && mfg[2] == 0x12) return "Apple Find My";
            return "Apple device";
        }
        if (company_id == 0x0006 && mfg_len >= 5 && mfg[2] == 0x03 && mfg[3] == 0x00 && mfg[4] == 0x80) {
            return "Windows Swift Pair";
        }
        if (company_id == 0x0075) return "Samsung device";
    }

    uint8_t svc_len = 0;
    const uint8_t *svc = find_ad(adv_data, adv_len, 0x16, &svc_len);
    if (svc != nullptr && svc_len >= 2) {
        uint16_t svc_uuid = svc[0] | (svc[1] << 8);
        if (svc_uuid == 0xFE2C) return "Fast Pair";
        if (svc_uuid == 0xFEED || svc_uuid == 0xFD84) return "Tile tracker";
    }

    return nullptr;
}

} // namespace BleClassify
