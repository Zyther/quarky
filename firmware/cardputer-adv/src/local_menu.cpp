#include "local_menu.h"
#include "hal/device.h"
#include <Arduino.h>

static const char *kItems[] = {"Ping (local test)", "About"};
static int s_selected = 0;

void LocalMenu::init() {
    Serial.println("quarky-cardputer-adv: local menu ready (standalone mode)");
}

void LocalMenu::tick() {
    // Real keyboard polling reads TCA8418 interrupt/registers via Device;
    // this bring-up version proves the menu loop runs without crashing.
    // Phase 4+ replaces this stub with real ';'/'.' up/down navigation
    // and Enter/Backspace select/back, matching UniGeek's nav convention.
}
