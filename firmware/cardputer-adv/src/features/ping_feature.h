#pragma once
#include <proto.h>
#include "../hal/ic2link.h"

namespace PingFeature {
void register_module();
void handle_start(IC2Link &link, uint16_t seq);
}
