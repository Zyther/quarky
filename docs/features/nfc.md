# NFC / RFID2

Tab5-native contactless-card tooling (NFC category, launcher tab "NFC"). Two
physically separate readers live behind this one tab, both wired to the same
HY2.0 PORT.A connector:

- **NFC unit** (ST25R3916): real ISO14443-A poller plus Listen Mode
  (tag emulation). Used by "NFC: Tag Read", "NFC: EMV Card Read", "NFC: Tag
  Library"'s Emulate action, and the emulation screen itself.
- **RFID2 unit** (WS1850S, register-compatible with MFRC522): used by
  "RFID2: Tag Read", "RFID2: MIFARE Keys", and "RFID2: Amiibo (NTAG21x)".

Only one physical unit can be plugged into PORT.A at a time, so only one of
the two readers is actually present on any given boot -- pick the tile that
matches whichever unit is plugged in.

## Shared hardware: PORT.A is one connector, arbitrated

PORT.A's GPIO53/54 pair is shared, at runtime, between the external I2C bus
(both NFC and RFID2 units), RF433 (scan/replay), and the IR unit, via
`Gpio53Arbiter`. Opening any NFC/RFID2 screen claims the bus; leaving the
screen releases it. If RF433 currently holds the pin (an active RF433
Scan/Capture session), every NFC/RFID2 bring-up in this category fails with
a status message and does not touch the hardware -- close the RF433 screen
first. The reverse is also true: an NFC/RFID2 session in progress will cause
RF433 to refuse to start.

Every screen below does one-shot chip bring-up the first time you tap its
"Scan"/"Read Tag" button, not on every poll. If bring-up fails (no unit
plugged in, or a bad connection), the screen **latches** into a failed state
with no automatic retry -- leave and re-enter the screen to try again.

## NFC: Tag Read / RFID2: Tag Read

Two nearly identical screens (`nfc_read.cpp`), one per physical unit.

1. Open **NFC > NFC: Tag Read** (NFC unit) or **NFC > RFID2: Tag Read**
   (RFID2 unit).
2. Tap **Scan**. The status label goes `Bringing up unit...` then
   `Scanning...`, polling for a tag every 250ms.
3. Present a tag. On success the result label shows the tag's type name and
   UID (colon-separated hex, e.g. `04:A3:F1:D2`). Tap **Scan** again to
   read another tag.
4. Tap **Save to Library** to write the just-read tag into the NFC Tag
   Library (below). This button re-checks that a tag is actually present in
   memory at click time -- if you tap Scan again before saving, the older
   tag is gone.

| Status text | Meaning |
|---|---|
| `Idle` | Screen just opened, nothing scanned yet |
| `Bringing up unit...` | One-shot chip init in progress |
| `Scanning...` | Polling for a tag, ~4 times/second |
| `Tag found` | A tag was read; result label shows type + UID |
| `Multiple tags -- present one at a time` | Collision during anticollision; present only one tag |
| `Tag answered but the exchange failed` | Protocol error mid-read; NFC unit only |
| `RFID2/NFC unit not responding` | Bring-up failed -- latched, re-enter screen to retry |

**What gets captured differs by unit.** The RFID2 path gets a real PICC-type
name (from the vendored MFRC522 library's own type table) and the UID/SAK,
but never a real ATQA -- the donor library doesn't expose it, so it's stored
as the documented `{0,0}` "not captured" sentinel. The NFC-unit path gets
the real UID/SAK/ATQA from the raw ISO14443-3A exchange, and additionally --
only for tags whose SAK is `0x00` (MIFARE Ultralight / NTAG21x family) --
captures the tag's actual page content (`GET_VERSION` plus every readable
page, up to 231 pages/NTAG216's real capacity) directly into the saved
record. This capture adds real time to that one scan: roughly 50ms for an
NTAG213, 140ms for an NTAG215, 240ms for an NTAG216. This page content is
what makes "Emulate" (below) able to answer a reader's actual read commands
instead of only its anticollision. A tag scanned on the RFID2 unit, or any
non-Type-2 tag (MIFARE Classic, EMV cards), never gets page content --
`page_count` stays 0.

## NFC: Tag Library

Browse, view, emulate, and delete tags saved from any of this category's
scan screens (`nfc_tag_library_ui.cpp`). Saved as one `.tag` file per tag
under `/quarky/captures/nfc/`, named by the tag's own UID hex -- re-saving
the same physical tag overwrites its prior entry rather than piling up
duplicates.

1. Open **NFC > NFC: Tag Library**. Up to 32 saved tags are listed.
2. Tap a row to load and view its details: type name, UID, and whether it
   carries captured page content ("captured pages -- full emulation" vs.
   "none -- UID/SAK/ATQA emulation only").
3. Tap **Emulate** to push the [Emulate Tag](#emulate-tag) screen for the
   currently-shown tag (NFC unit only -- see that section).
4. Tap **Remove** to delete the currently-shown tag's file from SD and
   refresh the list. Remove works even if the tag's record failed to load
   (a stale/corrupt file from an earlier format version) -- it's keyed by
   the row's filename, not by a successfully-parsed record, specifically so
   a broken entry can still be cleared out.

A record saved before 2026-08-24's page-content extension is 3 bytes
shorter than the current on-disk layout and will fail to load with "Failed
to load tag record" -- re-scan and re-save the tag on "NFC: Tag Read" to
get a loadable, page-content-capable record.

## Emulate Tag

Not a launcher tile -- reached only by tapping **Emulate** on a saved tag in
the Tag Library, and only meaningful for the NFC unit (`nfc_emulate.cpp`).
The RFID2/WS1850S unit's donor command set has no card-emulation command at
all, so this screen always drives the NFC unit's Listen Mode regardless of
which unit originally scanned the saved tag.

1. From the Tag Library, tap a saved tag, then tap **Emulate**.
2. The screen arms Listen Mode on the next tick with the tag's UID/SAK/ATQA
   (and page content, if the record has any). Status moves through:

| Status | Meaning |
|---|---|
| `Arming...` | One-shot Listen Mode setup in progress |
| `Waiting for a reader...` | Armed; nothing has approached yet |
| `Reader completed anticollision + SELECT...` | A reader found the emulated tag |
| `Reader is READING this tag!` | A reader sent a real Type-2-Tag READ and got real captured page content back -- the only state that confirms emulation is actually working end to end |
| `NFC unit stopped responding` | Hardware error -- latched, re-enter to retry |

If the saved tag has no captured page content (`page_count == 0` --
anything scanned on the RFID2 unit, anything not a Type-2 tag, or any
record saved before page capture existed), the status line says so
explicitly: a real reader will complete anticollision and SELECT, then get
NAKed on every subsequent read and typically keep re-polling. This is a
known, disclosed limit of UID-only emulation, not a bug -- re-scan the
physical tag on "NFC: Tag Read" to capture its pages if you need full
emulation.

Emulation is read-only against the reader: there is no ISO14443-4/T=CL
emulation, no MIFARE Classic emulation, and no Type-2-Tag WRITE support.

## RFID2: MIFARE Keys

MIFARE Classic key-recovery (`nfc_mifare_crack.cpp`), RFID2 unit only. Hold
one MIFARE Classic card on the reader for the whole run.

1. Open **NFC > RFID2: MIFARE Keys**.
2. Pick a mode (4 checkable buttons, mutually exclusive):

| Mode | What it actually does | Caveats |
|---|---|---|
| **Dictionary** | Tries every built-in key (36 common/default MIFARE keys) plus any `.txt` dictionary file under `/quarky/dict/nfc` against every sector/key-type | Straightforward; run this first |
| **Nested** | Uses one already-known key (from a prior Dictionary run) to recover keys for other sectors via the nested-authentication nonce attack | Needs a known key first; refuses otherwise |
| **Static nested (long shot)** | Same idea as Nested, for cards whose tag nonce doesn't change between authentications | Only works against static-nonce cards; if the card's nonce isn't actually static it reports failure rather than a wrong key |
| **Parity-oracle dictionary** | A dictionary attack pre-filtered by a parity oracle, attacking sector 0 key A only | Named "Darkside" in the donor project this was ported from, but it is **not** the real Courtois darkside attack (which needs no key list at all) -- it only ever returns a key already in your dictionary/built-ins |

3. Tap **Start**. A worker task runs on the Tab5's otherwise-idle second
   core so the UI keeps responding; progress bar, tried/total counts, and
   elapsed time update roughly 4 times/second. Tap **Stop** to cancel.
4. Recovered keys accumulate in the "Recovered keys" list as `S<n> A/B
   <hex key>` and survive between runs on the *same* card (needed so Nested
   modes have a key to exploit). Presenting a different card discards
   stale results automatically.
5. Tap **Save Keys** to write every recovered key for the current card to
   `/quarky/captures/mifare_keys/<UID hex>.txt`, one deduplicated
   colon-separated hex key per line -- in the same format the Dictionary
   mode's SD dictionaries accept, so a saved result can be dropped straight
   back into `/quarky/dict/nfc` for a future run.

**Real time cost, disclosed honestly by the UI itself.** A key-recovery
candidate walk (Nested/Static-nested) can legitimately need to check tens
of thousands of candidate keys against the physical card, each a ~150ms
authenticate-and-reset round trip. When the collected data doesn't
software-filter the candidate list (the "unfiltered" case, most likely on
Static Nested with only one usable sample), the walk is capped at 256
on-card verifications and reports "gave up" rather than running for hours
against a list that can hold up to 262,144 candidates.

## NFC: EMV Card Read

Read-only contactless payment-card reader (`nfc_emv_read.cpp`), NFC unit
only. Sends no `GENERATE AC`, computes no cryptogram -- extracts PAN,
expiry, and card vendor only.

1. Open **NFC > NFC: EMV Card Read**.
2. Tap **Scan**, then present a contactless payment card. The whole read
   (SELECT PPSE, SELECT AID, GET PROCESSING OPTIONS, walk the AFL with READ
   RECORD) runs in one shot with a 2.5-second overall budget.
3. On success, the result shows vendor (from a built-in AID dictionary, or
   `Unknown vendor (AID ...)`), PAN, and expiry.
4. **Save .nfc (Flipper)** writes a real Flipper Zero "Flipper NFC device"
   file (`.nfc`, ISO14443-4A device type) to `/quarky/captures/nfc/<UID
   hex>.nfc`, loadable by real Flipper-compatible tooling.
5. **Save to Tag Library** saves the card's UID/SAK/ATQA into the same NFC
   Tag Library used by "NFC: Tag Read" (type name prefixed `EMV <vendor>`),
   which means a saved EMV card is immediately emulatable via the Tag
   Library's Emulate button.

If a read fails partway (common causes: the card is not an ISO14443-4
contactless card at all, or it rejects GET PROCESSING OPTIONS with a
non-success status word), the status line reports the real EMV status
word/reason rather than a generic failure, and the screen stays in
scanning mode -- lift the card and re-present it to retry, matching the
natural "tap again" gesture.

## RFID2: Amiibo (NTAG21x)

Read/write raw page dumps of Ultralight/NTAG21x-family tags
(`nfc_amiibo.cpp`), RFID2 unit only. Named for its most common use
(Amiibo figures use NTAG215), but works on any Ultralight-family tag.

1. Open **NFC > RFID2: Amiibo (NTAG21x)**.
2. Tap **Read Tag**, present a tag. Reading proceeds 4 pages (16 bytes) at a
   time until the tag NACKs (real end-of-memory) or the donor's own 256-page
   loop bound is hit. The result shows the UID, a recognised chip name if
   the tag's Capability Container byte matches a known value (NTAG213/215/
   216), and the page count actually captured.
3. Tap **Write Dump Back** to write the in-memory dump back onto a
   (possibly different) tag. Pages 0-3 (UID/internal/lock/CC) and the last
   5 pages (dynamic lock/config/password/PACK) are never rewritten --
   only the user-data pages in between. Refuses if fewer than 10 pages were
   captured (not enough real writable pages to make writing meaningful).

**A real-hardware quirk worth knowing:** genuine Amiibo-programmed NTAG215
tags do not carry a standard Capability Container -- Nintendo's proprietary
data overwrites that page. When the CC byte isn't recognised, this screen
falls back to guessing the chip name from the real observed page count
(45/135/231 -> NTAG213/215/216) instead, which is confirmed reliable
against real Amiibo tags specifically because it doesn't depend on the
region Amiibo data overwrites.

The dump lives in RAM only for the current screen session -- there is no SD
save/load for this screen's dump (unlike the other NFC screens' tag-library
integration). Leaving and re-entering the screen, or tapping Read Tag again,
discards whatever was previously read.
