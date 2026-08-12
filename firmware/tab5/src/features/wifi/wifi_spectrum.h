#pragma once
namespace WifiSpectrumFeature {
void register_module();
void start();
// Called every loop() iteration while the spectrum screen is open; no-op
// otherwise. Advances the channel-hop + RSSI-sample state machine per
// Global Constraints' "no blocking calls > 50ms" rule.
void poll();
}
