#!/bin/bash
# ./build.sh        compile and refresh the update file in web/firmware
# ./build.sh flash  same, then flash over the cable (stick in download mode)
cd "$(dirname "$0")" || exit 1
FQBN=m5stack:esp32:m5stack_sticks3
arduino-cli compile --fqbn "$FQBN" --output-dir super_remote/build super_remote || exit 1
VER=$(grep -o '#define FW_VERSION "[^"]*"' super_remote/super_remote.ino | cut -d'"' -f2)
mkdir -p web/firmware
cp super_remote/build/super_remote.ino.bin web/firmware/revenant.bin
SIZE=$(stat -f%z web/firmware/revenant.bin)
printf '{"version":"%s","file":"revenant.bin","size":%s}\n' "$VER" "$SIZE" > web/firmware/version.json
echo "web/firmware now holds version $VER ($SIZE bytes)"
if [ "$1" = flash ]; then
  arduino-cli upload --fqbn "$FQBN" -p /dev/cu.usbmodem1101 --input-dir super_remote/build super_remote
fi
