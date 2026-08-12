#pragma once
namespace WifiPmkidFeature {
void register_module();
void start();
// Called every loop() iteration; no-ops unless the capture screen is open.
// Drains whatever the promiscuous-mode rx callback has queued into the ring
// buffer (bounded per call -- see wifi_pmkid.cpp) and appends it to the pcap
// file on SD, per Global Constraints' "no blocking calls > 50ms" rule. Also
// refreshes the on-screen packet counter.
void poll();
}
