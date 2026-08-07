#pragma once

enum class Category { WIFI, BLE, SUBGHZ, NRF24, LORA, NFC, RF433, IR, UTILITY };
enum class Affinity { TAB5_NATIVE, CARDPUTER_ADV, C5_NODE };

struct FeatureModule {
    const char *id;      // stable identifier, e.g. "wifi_scan"
    const char *name;    // display name for the launcher tile
    Category category;
    Affinity affinity;
};
