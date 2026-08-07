#pragma once
#include "iradio.h"

// WiFi backend for the ESP32-P4, which has no native radio of its own.
//
// The P4 reaches WiFi through the Tab5's onboard ESP32-C6 co-processor over
// SDIO, using Espressif's esp-hosted transport (WiFiRemote). Concretely, for
// this pioarduino/Arduino-ESP32 v3.3.11 build:
//   - The esp-hosted host-side component (espressif/esp_hosted, pinned via
//     the framework's dependencies.lock to the 2.x line) ships precompiled
//     into framework-arduinoespressif32-libs for the esp32p4 target -- see
//     <framework>/esp32p4*/sdkconfig, which already has
//     CONFIG_ESP_HOSTED_ENABLED=y and CONFIG_ESP_WIFI_REMOTE_ENABLED=y baked
//     in. There is no separate lib_deps/idf_component.yml entry to add for
//     it in this project; it is not exposed as an app-level dependency.
//   - arduino-esp32's WiFiGenericClass::wifiLowLevelInit() (see
//     libraries/WiFi/src/WiFiGeneric.cpp) already calls hostedInitWiFi()
//     under `#if CONFIG_ESP_HOSTED_ENABLED` before bringing the radio up, so
//     the standard <WiFi.h> API works unmodified on this target -- the
//     hosted/SDIO bring-up is transparent to application code.
// See .superpowers/sdd/2026-08-06-tab5-foundation-plan/task-9-report.md for
// the full research trail and what remains to be verified on real hardware.
class RadioEspHosted : public IRadio {
public:
    bool connect_wifi(const char *ssid, const char *pass) override;
    bool is_connected() override;
    uint32_t local_ip() override;
};
