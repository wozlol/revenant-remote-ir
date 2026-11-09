# Revenant plan

Master remote on an M5StickS3. Hears one remote, sends another. Web tool over Bluetooth for setup. Settings live in the stick and as a JSON file.

## Folders

- `ir_tester/` raw IR tester, done and working with both test remotes
- `super_remote/` stage 1 firmware
- `web/` web tool, run `./serve.sh` and open http://localhost:8090 in Chrome

## Stage 1, learn and send (built, needs the first real test)

- Stick: hears IR, sends IR, talks to the web tool over Bluetooth, saves settings to flash. Front button sends the last heard code back out. Right button opens the menu. Power button double click is full off. Silent.
- Web tool: connect, live monitor with green new and amber same, button grid with drag and drop, learn a button, online code library (Flipper-IRDB), built in Apple Remote Classic, settings save and load (stick, JSON file), demo mode with no stick.
- Apple Remote Classic uses remote ID 0x8B. An unpaired Apple TV accepts any ID.
- Library formats the web tool can send: NEC, NECext, Samsung32, SIRC, SIRC15, SIRC20, RC5, raw. Others show as unavailable.

## Test order

1. Connect, tap Apple TV buttons, watch the Apple TV react.
2. Search the library for the Sony receiver, add volume and mute, test on the receiver.
3. Search the library for Zenith, test on the TV. If nothing matches, learn from the real remote.
4. Learn a Panasonic button, send it back, confirm the TV reacts.

## Stage 2, Bluetooth keyboard (built, first test now)

- Stick advertises the settings link and a Bluetooth keyboard, media keys and mouse together. Just Works pairing, no code. Right menu has Forget All to clear pairings.
- Web tool has a built in Bluetooth Keyboard device with arrows, Enter, Backspace, Delete, Escape, Space, Tab, media, TV Home and Back, click and scroll. Add Key has a filter box, Enter adds the first match.
- Test: connect the stick in Chrome, pair it on the Apple TV (Settings, Remotes And Devices, Bluetooth, Revenant Keyboard), tap keys on the page.
- Unknown until tested: whether tvOS accepts this keyboard and which media keys it honors.

## Firmware updates (built)

Stick takes a new app image over Bluetooth (`OTA size`, raw chunks, acks every 4 KB). Web tool Stick Firmware card compares the stick version with `web/firmware/version.json` here and the online repo (`REMOTE_FW` in `web/index.html`, empty until the repo exists). Higher version wins, this folder wins ties. Needs one cable flash of 0.3 or later first.

## Stage 3, map IR in to out (built, 0.4, first test now)

Map Mode (arrow icon). Press a remote button, or tap a learned one, then tap the destination buttons. Destination device tabs remember the last device. Pairs live on the button in the settings file. Saving to the stick also sends a compiled rule table (`MAPBEGIN`, `MAPR` lines, `MAPEND`) that the stick keeps in `/map.txt` and runs by itself. Each rule can repeat while the remote button is held. Destinations can be IR or Bluetooth keys, but keys only work once the Bluetooth keyboard is fixed.

## Wi-Fi (built, 0.5, first test now)

Wi-Fi is set up once over Bluetooth (`WIFI ssid|pass`, saved in `/wifi.txt`). The stick then runs a small web server (`/hello`, `/cmd`, `/poll`, `/ota`) with CORS, and answers `revenant.local`. The page switches to Wi-Fi, polls for heard buttons, and sends firmware as one upload. Chrome needs the page on http (the local server) to reach the stick, https pages are blocked.

## Order from here

1. Test Wi-Fi join, then fast updates
2. Redo the Space Command capture (all frames and timings from the stick), then fix its identification. The original Space Command is ultrasonic, so the model in hand needs a data first approach, not a protocol guess. Old remotes may skip the 38 kHz carrier, so pulse widths can vary
3. Fix Bluetooth to the Apple TV. With Wi-Fi for settings, Bluetooth can be keyboard only, with the settings service kept only for first time Wi-Fi setup and recovery
4. Map to Bluetooth keys and test on the Apple TV

## Stage 4, voice

Two modes, each with its own trigger button. Online mode uses Wi-Fi and a speech to text service for words like backspace and enter. Local mode uses no Wi-Fi and knows the alphabet, arrows, backspace, delete, clear and enter. Both type through the Bluetooth keyboard. Needs a service and key chosen for online mode, and a model chosen for local mode.

## Known limits

- Backlight flicker leaks into the IR receiver. Pulses under 300 us are dropped and frames need at least 9 timings or a long first pulse.
- Web Bluetooth needs Chrome.
- Not yet tested on hardware: Bluetooth link, send, settings save, keyboard pairing.
