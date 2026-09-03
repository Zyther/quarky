# Pair Satellite

Shows the pre-shared key (PSK) used to pair the Tab5 with a Cardputer-ADV
satellite, and lets you opt into the WiFi C2 transport.

## Using it

1. Open **Pair Satellite** from the launcher (a standalone tile, not under
   any category).
2. The screen shows a QR code and, underneath it, the same key as a
   32-character hex string. **Use the hex string** — the Cardputer-ADV has
   no camera, so the QR code only matters for some future camera-equipped
   companion device. Read the hex string off the screen (or off the serial
   log, which prints the same value) and enter it on the satellite side.
3. A PSK is generated once and persisted; reopening this screen later shows
   the same key rather than generating a new one, so pairing is a one-time
   setup unless you deliberately reset it.

## WiFi C2 (optional)

BLE C2 is the default transport and is always available. WiFi C2 is off by
default because bringing it up costs a real ~146KB of this board's limited
DMA-capable memory pool whether or not you're using it — memory other
SD-heavy features need.

If you specifically need WiFi C2's higher throughput, tap **Enable WiFi
Link** on this screen. The label updates to `WiFi C2: enabled` on success,
or shows a failure reason if it couldn't start. Once enabled it stays up
for the rest of the boot; there's no way to turn it back off short of
rebooting. If it's already enabled, this screen shows `WiFi C2: enabled`
with no button at all.
