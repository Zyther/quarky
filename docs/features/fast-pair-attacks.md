# Fast Pair Exploit & WhisperPair — reading the results

Two Tab5-native BLE features target Google Fast Pair accessories (BLE service
UUID `0xFE2C`) — earbuds, speakers, trackers and the like:

- **Fast Pair Exploit** — connects and writes hand-crafted payloads at the
  Key-based Pairing (KBP) characteristic, looking for memory corruption.
- **WhisperPair (CVE-2025-36911)** — connects, subscribes to KBP notifications,
  writes one encrypted KBP request while the accessory is *not* in pairing
  mode, and watches whether the accessory answers.

Both are **status-line tools**: they report what happened, not a verdict badge.
This page is how to read those lines, because "nothing happened" means several
very different things.

## Pick the target first

Both features run against **BLE scan slot 0** — the first device the last
**BLE > BLE Scan** run saw, which is very often somebody's phone rather than a
Fast Pair accessory. There is no target picker yet (real, considered future
work; see the notes in `ble_fastpair_exploit.cpp`'s `register_module()`).

Practically: run **BLE Scan** first, confirm the device you want is the first
row, then open one of these screens. If no scan has been run at all, the screen
says so instead of doing nothing.

Opening either screen with no scan run, or with the wrong device in slot 0, is
the single most common cause of a misleading result.

## WhisperPair result lines

| What you see | What it means |
|---|---|
| `NOTIFY on KBP -- target responded outside pairing mode (CVE-2025-36911 indicated)` | **Positive.** The accessory serviced a KBP request while not in pairing mode. This is the actual CVE signal. |
| `no reply within window -- target stayed silent outside pairing mode (patched or not applicable)` | **Negative, and genuinely ambiguous** — see below. |
| `no Fast Pair service (0xFE2C) -- not applicable to this target` | **Wrong target**, not a security result. The device is not a Fast Pair accessory. |
| `KBP characteristic NOT found in Fast Pair service -- ...` | Fast Pair advertised but the KBP characteristic is absent. Inconclusive; treat as "not testable". |
| `connect failed status=N`, `svc discovery ... failed`, `... cannot subscribe` | **Inconclusive.** The test never ran to completion. Retry closer to the target; these are usually link failures, not findings. |

**Why a silent target is not proof of "patched".** The probe is encrypted with
a key the accessory cannot decrypt — this port does not extract the
accessory's own public key, so no real ECDH shared secret is established. A
patched device and a device that simply ignored an undecryptable blob look
identical from outside. What this feature establishes is narrower than
"vulnerable/not vulnerable": **does the accessory respond to a KBP write
outside pairing mode at all.** A response is meaningful. Silence is weak
evidence at best.

Only notifications arriving on the KBP characteristic's own attribute handle
count. Fast Pair accessories notify on other characteristics (Passkey, Account
Key) for unrelated reasons, and those are filtered out deliberately — a false
positive is the worst failure mode a detector can have.

## Fast Pair Exploit result lines

This one is manual: connect, then tap **Send overflow payload** (512 bytes via
a GATT long write) or **Send state-confusion payload** (8 bytes) and watch.

| What you see | What it means |
|---|---|
| `KBP characteristic found (handle N) -- ready` | Target is a Fast Pair accessory and is testable. Payload buttons are meaningful from here. |
| `no Fast Pair service (0xFE2C) on this target` | **Wrong target.** Not a result. |
| `... write started (rc=0)` | The write was accepted by the local stack and sent. This says nothing yet about the target's reaction. |
| `... write FAILED rc=N` | The write never went out. Inconclusive. |
| `disconnected (reason=N)` right after a payload | **The interesting case.** The accessory dropped the link when it received the payload. Suggestive of a crash or a defensive disconnect — not proof of either. |

There is no "vulnerable" line, on purpose. The observable outcomes are: the
accessory ignores the write, rejects it with an ATT error, or drops the
connection. Only the third is worth investigating further, and confirming what
actually happened inside the accessory (crash vs. reset vs. deliberate
disconnect) needs evidence this tool cannot collect — power-cycle behaviour,
whether it re-advertises, whether pairing state was lost.

## Both features, general caveats

- Serial output carries strictly more detail than the on-screen line (every
  NimBLE return code is logged). If a result looks ambiguous, read the serial
  log before drawing a conclusion.
- These connect to the target and hold the link. Leaving the screen tears the
  connection down thoroughly (the teardown burns a fixed 500 ms cancelling
  in-flight connects, on purpose — see the code comments) so the next BLE
  feature does not inherit a leaked connection slot. A Back tap that seems to
  hang for half a second is that, working as intended.
- Use only against devices you own or are authorized to test.
