# BLE

Tab5-native Bluetooth Low Energy tools (BLE category launcher tiles). All of
these run on the Tab5's own BLE radio (proxied to the ESP32-C6 over
esp-hosted/SDIO) — none of this requires the Cardputer-ADV. Two screens in
this category are documented separately rather than here:

- **BLE Bad-KB** → [`ble-bad-kb.md`](ble-bad-kb.md)
- **Fast Pair Exploit** and **WhisperPair** → [`fast-pair-attacks.md`](fast-pair-attacks.md)

## Recon

### BLE Scan

General-purpose BLE device scanner. Runs an active scan for 10 seconds and
lists every device seen, most-recently-updated in place (not re-sorted).

**Using it**

1. Open **BLE > BLE Scan**. Scanning starts immediately.
2. Each row shows the device's advertised name (or its address if it has
   none), an optional classification tag in brackets, and RSSI in dBm.
3. After 10 seconds a final row is appended: `Scan complete (N devices)`.
4. Tap **Back** to cancel early (safe to do — a cancel racing the scan's own
   natural end is logged, not treated as an error) and leave.

The classification tag comes from parsing the advertisement's
manufacturer-data/service-data fields, not from the device name:

| Tag shown | What triggered it |
|---|---|
| `iBeacon` | Apple mfr ID, iBeacon subtype |
| `AirPods (Continuity)` | Apple mfr ID, Continuity subtype |
| `Apple Nearby Action` | Apple mfr ID, Nearby Action subtype |
| `Apple Find My` | Apple mfr ID, Find My subtype |
| `Apple device` | Apple mfr ID, unrecognized subtype |
| `Windows Swift Pair` | Microsoft mfr ID + Swift Pair subtype bytes |
| `Samsung device` | Samsung mfr ID |
| `Fast Pair` | Fast Pair service UUID (0xFE2C) in service data |
| `Tile tracker` | Tile service UUID (0xFEED/0xFD84) in service data |

No tag at all just means the advertisement didn't match any of the above —
the device is still listed by name/address and RSSI.

### BLE Sniffer (CSV)

Passive scan that writes every advertisement seen to a CSV capture file on
the SD card, instead of showing a live device list. Meant for offline
analysis rather than on-screen triage.

**Using it**

1. Open **BLE > BLE Sniffer (CSV)**. A new file is created at
   `/quarky/captures/ble/sniff_<millis>.csv` and scanning starts immediately
   (no time limit — it runs until you leave the screen).
2. The status line reads `Capturing... N rows (M dropped)`, updated live.
3. Tap **Back** to stop scanning and close the file.

Each row is `ms,mac,rssi,addr_type,raw_adv_hex` — the full raw advertisement
payload as hex, not a parsed name/classification. `N` is rows successfully
queued; `M` is rows dropped because the internal write buffer filled up
faster than the SD card could drain it (a 4KB ring, drained up to 2KB per
main-loop tick) — a nonzero drop count in a very BLE-dense environment means
the capture has gaps, not that anything is broken.

### BLE Tracker Finder

Detects known Bluetooth trackers (AirTag, Samsung SmartTag, Tile) and, once
you lock onto one, switches to a proximity ("geiger counter") display driven
by that tracker's live RSSI.

**Using it**

1. Open **BLE > BLE Tracker Finder**. A passive scan starts and the list
   fills with `Kind  (address)  RSSI dBm` rows as trackers are detected —
   before anything is found it reads "Scanning for trackers...".
2. Tap a row to lock onto that specific tracker. The scan keeps running (it
   has to, to keep receiving that tracker's advertisements) but the screen
   switches to proximity mode.
3. The proximity line shows a distance tier and the current RSSI:

   | Tier | Meaning |
   |---|---|
   | `RIGHT HERE` | RSSI above -45 dBm |
   | `HOT` | -45 to -60 dBm |
   | `WARM` | -60 to -72 dBm |
   | `COOL` | -72 to -84 dBm |
   | `COLD` | below -84 dBm |
   | `NO SIGNAL` | no sighting of the locked tracker in the last 4 seconds |

4. Tap **Back** to stop scanning and leave (there is no way to unlock and
   return to the list without leaving the screen and reopening it).

Detection is signature-based: AirTag = Apple manufacturer ID with the Find My
subtype byte; SmartTag = Samsung manufacturer ID; Tile = Tile's service UUID.
It only recognizes these three families, not trackers in general.

### GATT Explorer

Scan-then-connect GATT enumeration: pick a nearby device and see its full
service/characteristic tree.

**Using it**

1. Open **BLE > GATT Explorer**. A 10-second active scan runs and populates a
   tappable list of `name-or-address  RSSI dBm` rows.
2. Tap a row to connect (5-second connect timeout). The screen switches from
   the device list to a discovery log.
3. The log appends as discovery proceeds: `svc <uuid>` for each service,
   `  chr <uuid> (handle N)` indented under it for each characteristic, and
   `-- discovery complete --` at the end. A failed connect appends
   `connect failed status=N` instead.
4. Tap **Back** to disconnect and leave. If Back is tapped while the connect
   attempt is still in flight, teardown spends up to 500ms making sure the
   connection doesn't land and get orphaned after the screen is gone — a
   half-second pause on Back here is expected, not a hang.

This is a read-only enumeration tool — it does not read characteristic
values, only lists what exists and at what handle.

## Spoofing and flooding

Use only against devices you own or are authorized to test.

### BLE Clone

Captures a nearby device's advertised name and identity and re-broadcasts
the Tab5 as a connectable copy of it.

**Using it**

1. Open **BLE > BLE Clone**. A 10-second active scan runs, filtered to
   devices using a **random** BLE address with a non-empty advertised name
   (public-address devices are skipped — "public addrs unclonable" per the
   source comment, since impersonating a fixed public MAC isn't practical to
   fake convincingly the same way).
2. Tap a row (`name  address`) to clone it. The Tab5 sets its own BLE
   identity to a masked copy of that device's MAC and starts advertising
   under its name, connectable.
3. Tap **Back** to stop advertising and leave.

**Side effect:** cloning sets the Tab5's host-wide random BLE identity to the
cloned target's MAC. There is no API to "unset" it — it persists until reboot
or until another feature overwrites it. See "Identity side effects" below.

### BLE Karma

Passively watches for any nearby BLE traffic and, once traffic is seen,
rotates the Tab5's own advertised identity and name every 2 seconds through a
list of common consumer-device names (AirPods Pro, Galaxy Buds Pro, Sony
WH-1000XM4, Echo Dot, and others).

**Using it**

1. Open **BLE > BLE Karma**. A passive scan starts; the status line reads
   "Starting..." until the first nearby advertisement is seen.
2. Once any BLE traffic is detected, identity rotation begins. The status
   line updates to `Rotating identity (<name> next)` on every rotation.
3. Tap **Back** to stop scanning/advertising and leave.

This rotates whenever *any* BLE advertisement is seen nearby — it is not
triggered specifically by a scan request addressed to this device (NimBLE's
scan callback doesn't expose that distinction). In a quiet room with no other
BLE traffic, Karma never rotates and never takes the advertising slot at all.

**Side effect:** each rotation sets a fresh random host-wide BLE identity
with no unset API — the most frequent churn of any BLE screen while it's
active. See "Identity side effects" below.

### Sour Apple (CVE-2023-42941)

Floods rotating spoofed pairing-popup advertisements covering seven distinct
vendor formats, triggering "new device" pairing sheets on nearby phones.

**Using it**

1. Open **BLE > Sour Apple**. Flooding starts immediately — there is no
   target picker or configuration; the status line reads "Flooding...".
2. Tap **Back** to stop.

It cycles sequentially through all seven modes at 200ms per send (so a full
rotation takes 1.4s): Apple ProximityPair popup ("new AirPods"), Apple
Nearby Action (Setup New iPhone / AppleTV pairing prompts), an AirTag-style
variant, Samsung EasySetup (Buds), Samsung EasySetup (Watch), Microsoft Swift
Pair, and Google Fast Pair. Each send uses randomized device-model/battery/key
fields within that mode's real format, not a single fixed payload.

**Side effect:** every single send (5 times per second) also re-randomizes
the Tab5's own BLE identity, with no unset API. This is the heaviest identity
churn of any feature in this project — a one-minute run leaves roughly 300
discarded identities behind it. See "Identity side effects" below.

### Find My Emulator

Broadcasts a single simulated Apple Find My accessory (like a lost AirTag),
slowly rotating its identity to mimic a real low-power accessory rather than
flooding.

**Using it**

1. Open **BLE > Find My Emulator**. Broadcasting starts immediately; the
   status line reads "Broadcasting...".
2. The Tab5 rotates to a fresh simulated tag identity/key every 60 seconds.
   There's no on-screen indication of exactly when a rotation happens beyond
   the label staying "Broadcasting...".
3. Tap **Back** to stop and leave.

The advertisement uses a deliberately slow advertising interval (matching a
real battery-powered accessory) and a random 22-byte value standing in for a
real Curve25519 public key — nearby iPhones don't validate that key, they
relay it blindly to Apple's offline-finding network, which is what makes the
emulation work without real Apple account/key infrastructure.

**Side effect:** each 60-second rotation sets a new random host-wide BLE
identity, no unset API — but unlike Sour Apple's rapid churn, the *specific*
address left behind here may plausibly have already been relayed by a
passing iPhone to iCloud as a real sighting, since that is the intended
behavior of the emulation. See "Identity side effects" below.

### BLE Spam

Broadcasts a single selectable vendor pairing-popup payload on repeat, rather
than rotating through several.

**Using it**

1. Open **BLE > BLE Spam**. A dropdown at the top lists the available
   payloads: **AirPods**, **Fast Pair**, **Swift Pair**, **Samsung**.
2. Pick a vendor from the dropdown at any time — the broadcast switches to
   it within one send cycle (200ms).
3. The status line reads "Spamming..." for the duration.
4. Tap **Back** to stop and leave.

Unlike Clone/Karma/Sour Apple/Find My, BLE Spam broadcasts under the Tab5's
own fixed public address — it does **not** call the identity-rotation API, so
it does not contribute to the identity-churn side effect described below.

### BLE Flood

Rapid connect-then-immediately-disconnect cycling against one chosen target,
meant to overwhelm peripherals (smart locks, simple accessories) that can
only track a small number of BLE connections at once.

**Using it**

1. Open **BLE > BLE Flood**. A scan-and-pick screen appears first — tap a
   discovered device to select it as the flood target.
2. Once picked, the flood begins: connect, disconnect the instant the
   connection lands, repeat, roughly every 250ms. The status line reads
   `Flooding... N attempts`, counting up.
3. Tap **Back** to stop. Teardown can take up to 500ms to guarantee no
   in-flight connection is left dangling on the target after you leave —
   this is deliberate, not a freeze.

No identity rotation — connections are made from the Tab5's fixed own
address (via the shared central-connect helper), so this is a connection-rate
attack, not an identity-spoofing one.

## Exploit / leak checks

Use only against devices you own or are authorized to test.

### HFP Exploit (foothold check)

Despite the tile name, this does not perform an HFP handshake or capture
audio — it connects to a target and checks whether it leaks two Classic
Bluetooth SDP service UUIDs (Handsfree / Handsfree Audio Gateway) over BLE
GATT, which normally shouldn't be reachable that way.

**Using it**

1. Open **BLE > HFP Exploit**. A scan-and-pick screen appears first — tap a
   device to connect to it (5-second connect timeout).
2. Read the result line once both service searches finish:

| What you see | What it means |
|---|---|
| `connected -- checking for HFP service leak (0x111E/0x111F)` | In progress. |
| `HFP service LEAKED over BLE: <uuid> [<label>], handles X-Y` | **Positive.** The target exposes a Classic-BT SDP service (Handsfree or Handsfree AG) over its BLE GATT database. |
| `no HFP service exposed over BLE (not vulnerable to this check)` | **Negative, and expected.** Essentially every modern device reads this way — it is a correct finding, not a broken feature. |
| `HFP check INCOMPLETE: N of 2 service searches errored -- result unknown` | Inconclusive — one or both searches failed to complete cleanly. |
| `connect failed status=N` / `connect could not start rc=N` | The connection itself never succeeded. Not a result. |
| `disconnected (reason=N)` | The link dropped — check whether a verdict line already printed before this. |

3. Tap **Back** to disconnect and leave. As with GATT Explorer, backing out
   during an in-flight connect attempt can take up to 500ms to tear down
   cleanly.

## Identity side effects

Four of the screens above — **BLE Clone**, **BLE Karma**, **Sour Apple**, and
**Find My Emulator** — set the Tab5's host-wide random BLE identity
(`ble_hs_id_set_rnd()`) as part of what they do, and this project's NimBLE
build has no supported way to unset it once set. Whatever address was current
when you leave one of these screens stays the Tab5's identity for the rest of
the boot — until a reboot, or until another feature overwrites it again.

| Screen | Rotation rate |
|---|---|
| BLE Clone | Once, to the cloned target's real MAC |
| BLE Karma | Every 2s, once nearby traffic triggers it |
| Sour Apple | Every 200ms (~5/sec) — heaviest churn |
| Find My Emulator | Every 60s |

**BLE Bad-KB is the feature this actually affects** — see
[`ble-bad-kb.md`](ble-bad-kb.md)'s own "Side effects on other features"
section. In short: if a host already bonded with the Tab5's `QuarkyKB`
keyboard identity, running any of the four screens above and then opening
Bad-KB in the same boot can make that host treat it as an unrecognized new
device and force a re-pair — and the bond store only holds 3 entries. Reboot
before a Bad-KB session that depends on an existing pairing.

## The single BLE advertising slot

This project has not configured BLE Extended Advertising, so exactly one
legacy advertisement can be live at a time, system-wide. **BLE Clone, BLE
Karma, Sour Apple, Find My Emulator, and BLE Spam** all advertise, which
means opening any of them stops the Tab5's own C2 advertisement
(`Quarky-Tab5`) for as long as the screen is open — the Tab5 is not
discoverable/connectable for BLE C2 pairing during that window. The C2
advertisement is automatically restored when you leave the screen, so this
is a for-the-duration outage, not a for-the-boot one. BLE Scan, BLE Sniffer,
BLE Tracker Finder, GATT Explorer, BLE Flood, and HFP Exploit are
scan/connect-only and never touch advertising, so they have no effect on C2.

## If BLE radios aren't up

Every screen in this category checks whether the BLE host actually came up
before touching it. On a boot where the radios are disabled (a real,
supported degraded mode — the launcher tile still shows even then), opening
any of these screens shows an explanatory "not ready" message instead of a
blank screen or a crash.
