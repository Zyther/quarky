#!/usr/bin/env bash
# build.sh
set -euo pipefail

echo "== Building firmware/tab5 =="
(cd firmware/tab5 && pio run)

echo "== Building firmware/cardputer-adv =="
(cd firmware/cardputer-adv && pio run)

echo "== Both targets built =="
echo "tab5:          firmware/tab5/.pio/build/tab5/firmware.bin"
echo "cardputer-adv: firmware/cardputer-adv/.pio/build/cardputer-adv/firmware.bin"
