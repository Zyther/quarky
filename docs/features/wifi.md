# WiFi

Tab5-native 2.4GHz WiFi tools (WiFi launcher category). All of these run over
the Tab5's onboard ESP32-C6 co-processor via esp-hosted (SDIO link) — there's
no external unit or HY2.0 connector involved, unlike NFC/RF433/IR, which
share the Tab5's one physical PORT.A connector. Every screen below shares the
C6's single radio with the WiFi C2 link (the SoftAP the Cardputer-ADV
satellite joins), so what each screen does to WiFi's AP/STA mode matters —
noted per screen.

For **Evil Portal** (captive-portal credential capture), see
[evil-portal.md](evil-portal.md) — it's WiFi-category but documented
separately.

## WiFi Scan

Launcher tile: **WiFi Scan**. Passive AP scan with live results list.

### Using it

1. Open **WiFi > WiFi Scan**. The screen opens showing "Scanning..." and a
   scan starts immediately.
2. Results populate automatically when the scan finishes — no button to
   press. Each row reads `SSID  chN  -NNdBm  OPEN` (the `OPEN` suffix only
   appears on networks with no auth/encryption).
3. Tap **Back** to leave. Reopening the screen starts a fresh scan; nothing
   is cached between visits.

Rows are in WiFi's own RSSI-descending order (strongest signal first). Up to
32 access points are shown; anything beyond that from a larger scan is not
displayed. The scan gives up after 15 seconds and shows "Scan failed or
timed out" if it hasn't completed by then; an empty result set shows "No
networks found". List rows are informational only — tapping one does
nothing (there's no drill-down or connect-from-list action).

This screen widens WiFi to `WIFI_AP_STA` if the C2 SoftAP is currently up (or
`WIFI_STA` if WiFi was off), so the C2 link is not dropped by opening this
screen.

## WiFi Spectrum

Launcher tile: **WiFi Spectrum**. Live per-channel RSSI bar chart across all
14 2.4GHz channels — the same kind of view a phone WiFi-analyzer app shows.

### Using it

1. Open **WiFi > WiFi Spectrum**. A 14-bar chart appears (channels 1–14,
   labeled underneath), y-axis fixed at -100 to 0 dBm.
2. The chart updates continuously while the screen is open — it hops one
   channel at a time (150ms dwell per channel), refreshing that channel's
   bar with whatever it heard before moving to the next. A full 14-channel
   sweep takes roughly a couple of seconds.
3. There's no start/stop control and no "sweep complete" state — it just
   keeps hopping and refreshing until you tap **Back**.

Each bar is the strongest beacon/probe-response RSSI heard on that channel
during its dwell window, not a true RF power measurement (that needs
different silicon) — same tradeoff any consumer WiFi-analyzer app makes. A
channel with no AP activity during its dwell shows -100 dBm (the chart's own
floor), which reads as "nothing heard" rather than "confirmed silent" — a
weak or infrequently-beaconing AP can be missed on any given pass.

Opening this screen applies the same AP/STA-preserving mode widening as WiFi
Scan (`WIFI_AP` → `WIFI_AP_STA`, or `WIFI_OFF` → `WIFI_STA`), so it does not
drop the C2 SoftAP either.

## WiFi Connect

Launcher tile: **WiFi Connect**. Joins the Tab5 to an existing WiFi network
as a station.

### Using it

1. Open **WiFi > WiFi Connect**.
2. Tap the **SSID** field to enter a network name — an on-screen keyboard
   pops up. Tap **Password** to enter the passphrase the same way.
3. Tap **Connect**. The status label changes to "Connecting...".
4. Wait — a connect attempt can take up to ~15 seconds (it retries against
   the AP before giving up). The status label updates in place when it
   finishes:

| Status text | Meaning |
|---|---|
| `Not connected` | Initial state, nothing attempted yet |
| `Connecting...` | Attempt in progress |
| `Connected (ip=x.x.x.x)` | Success — shows the assigned station IP |
| `Connect failed` | Wrong password, AP out of range, timeout, etc. — no further detail shown on-screen |

Tapping **Connect** again while an attempt is already in flight does
nothing — only one attempt runs at a time. There's no cancel button;
a slow/hung attempt just has to be waited out (or the screen backed out of
— see below).

### Side effects and limits

- SSID is capped at 32 bytes and password at 63 bytes (real 802.11/WPA2
  limits) — input beyond that is silently truncated.
- Like WiFi Scan/Spectrum, connecting widens the WiFi mode rather than
  replacing it: if the C2 SoftAP is up, the Tab5 goes to `WIFI_AP_STA`, so
  the Cardputer-ADV's C2 link is **not** dropped by using this screen.
- The connect attempt itself runs in a background task, not on the UI
  thread, so the screen (and Back) stay responsive while it's in progress.
  Backing out of the screen mid-attempt does not cancel it — the connect
  keeps running to completion in the background; its result is just
  discarded instead of being shown, since the status label it would have
  updated no longer exists.
