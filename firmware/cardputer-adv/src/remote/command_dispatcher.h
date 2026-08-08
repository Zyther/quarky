#pragma once
#include "../hal/ic2link.h"
#include <feature_registry.h>

// Routes incoming c2proto::Frame commands (received over IC2Link, from
// Tab5) to the local FeatureRegistry. Cardputer-ADV counterpart to whatever
// Tab5 uses to interpret RESP_* frames from this device.
//
// This is a routing skeleton, not a feature-complete dispatcher: Task 4's
// FeatureModule only carries id/name/category/affinity today, with no
// start/stop/telemetry function pointers yet. Task 20 (Ping feature) adds
// those and completes CMD_START_FEATURE/CMD_STOP_FEATURE handling against
// them.
namespace CommandDispatcher {
void handle(const c2proto::Frame &frame, IC2Link &link, FeatureRegistry &registry);
}
