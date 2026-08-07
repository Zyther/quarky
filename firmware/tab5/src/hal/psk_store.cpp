#include "psk_store.h"
#include <Preferences.h>

static const char *kNamespace = "quarky-c2";
static const char *kKey = "psk";

bool PskStore::load(uint8_t out[16]) {
    Preferences prefs;
    prefs.begin(kNamespace, true);
    size_t n = prefs.getBytes(kKey, out, 16);
    prefs.end();
    return n == 16;
}

void PskStore::save(const uint8_t psk[16]) {
    Preferences prefs;
    prefs.begin(kNamespace, false);
    prefs.putBytes(kKey, psk, 16);
    prefs.end();
}
