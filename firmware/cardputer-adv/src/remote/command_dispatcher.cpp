#include "command_dispatcher.h"
#include <Arduino.h>
#include <cstring>

namespace CommandDispatcher {

namespace {

// All categories FeatureRegistry knows about (feature_module.h). Task 4's
// FeatureRegistry only exposes for_each_in_category() and find_by_id() --
// no raw index-by-position accessor -- so a capability report is built by
// walking every category and accumulating the ids it finds.
constexpr Category kAllCategories[] = {
    Category::WIFI,   Category::BLE,  Category::SUBGHZ, Category::NRF24, Category::LORA,
    Category::NFC,    Category::RF433, Category::IR,     Category::UTILITY,
};

void append_id(char *buf, size_t buf_cap, size_t &offset, const char *id) {
    size_t id_len = strlen(id);
    size_t sep_len = (offset > 0) ? 1 : 0; // leading comma if not the first entry
    if (offset + sep_len + id_len >= buf_cap) return; // would overflow -- drop remaining ids silently
    if (sep_len) buf[offset++] = ',';
    memcpy(buf + offset, id, id_len);
    offset += id_len;
    buf[offset] = '\0';
}

} // namespace

void handle(const c2proto::Frame &frame, IC2Link &link, FeatureRegistry &registry) {
    if (frame.type == c2proto::MsgType::CMD_GET_STATUS) {
        // Capability report: comma-joined list of registered feature ids,
        // truncated to fit kMaxPayload -- Phase 2+ features register here
        // and automatically become visible to the Tab5's capability check.
        c2proto::Frame resp{};
        resp.version = c2proto::kVersion;
        resp.type = c2proto::MsgType::RESP_STATUS;
        resp.seq = frame.seq;

        char ids[c2proto::kMaxPayload + 1] = {0};
        size_t offset = 0;
        for (Category cat : kAllCategories) {
            registry.for_each_in_category(cat, [&](const FeatureModule &m) {
                append_id(ids, sizeof(ids), offset, m.id);
            });
        }

        size_t len = strlen(ids);
        if (len > c2proto::kMaxPayload) len = c2proto::kMaxPayload; // defensive, append_id already guards this
        memcpy(resp.payload, ids, len);
        resp.payload_len = (uint16_t)len;
        link.send(resp);
        return;
    }

    if (frame.type == c2proto::MsgType::CMD_START_FEATURE || frame.type == c2proto::MsgType::CMD_STOP_FEATURE) {
        if (frame.payload_len == 0 || frame.payload_len > c2proto::kMaxPayload) return; // malformed, ignore

        char feature_id[c2proto::kMaxPayload + 1] = {0};
        memcpy(feature_id, frame.payload, frame.payload_len);
        feature_id[frame.payload_len] = '\0';

        const FeatureModule *m = registry.find_by_id(feature_id);
        if (m == nullptr) {
            // Unsupported feature -- silently ignored. Tab5's capability
            // check (populated from this dispatcher's CMD_GET_STATUS
            // response) should have prevented this from being sent at all.
            return;
        }

        // m's start/stop callback is invoked here once Phase 2+ modules add
        // one to FeatureModule's struct; Task 4's contract only carries
        // id/name/category/affinity today, and Task 20 (ping) extends it
        // with actual start/stop/telemetry function pointers.
        Serial.printf("quarky-cardputer-adv: command_dispatcher: %s requested for feature '%s' "
                       "(no start/stop callback wired yet -- see Task 20)\n",
                       frame.type == c2proto::MsgType::CMD_START_FEATURE ? "start" : "stop", m->id);
        return;
    }
}

} // namespace CommandDispatcher
