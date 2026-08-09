#pragma once

enum class Category { WIFI, BLE, SUBGHZ, NRF24, LORA, NFC, RF433, IR, UTILITY };
enum class Affinity { TAB5_NATIVE, CARDPUTER_ADV, C5_NODE };

using FeatureStartFn = void (*)();
using FeatureStopFn = void (*)();

struct FeatureModule {
    const char *id;      // stable identifier, e.g. "wifi_scan"
    const char *name;    // display name for the launcher tile
    Category category;
    Affinity affinity;
    FeatureStartFn on_start = nullptr; // executor side only; nullptr on Tab5 descriptors for remote-affinity modules
    FeatureStopFn on_stop = nullptr;
};
