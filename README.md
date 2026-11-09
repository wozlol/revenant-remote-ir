# Revenant

IR Remote Router Relay Repeater & Bluetooth Keyboard Adaptor, on an M5StickS3. It hears IR remotes, sends IR codes, and shows up as a Bluetooth keyboard, media keys and mouse. A web page in Chrome sets it up over Bluetooth.

<p align="center">
  <img src="https://m5stack-doc.oss-cn-shenzhen.aliyuncs.com/1207/K150-stickS3_main-products_01.webp" alt="M5StickS3" width="380">
  <img src="https://static-cdn.m5stack.com/resource/docs/products/unit/ir/ir_01.webp" alt="M5 IR Unit" width="380">
</p>

<p align="center">M5StickS3 and the M5 IR Unit. Product photos from <a href="https://docs.m5stack.com/en/core/StickS3">M5Stack</a>.</p>

## Where to use it

Revenant serves its own web page. Once it is on your Wi-Fi, open **http://revenant.local** in Chrome (or the address on Revenant's screen) and the whole app is there, talking to Revenant over Wi-Fi.

**First time**
1. Open **https://woz.lol/revenant/** in Chrome. Click Connect Revenant, pick Revenant, and join your Wi-Fi in the Revenant Wi-Fi card. Bluetooth setup only works from a secure page like this one (or `localhost`).
2. Open **http://revenant.local**. Press the front button on Revenant, then click Connect. The page gets its key from Revenant for two minutes after that press and keeps it.

**Every time after**
- Open http://revenant.local. It connects by itself. Press the front button and click Connect again only if it says so, for example after Forget Wi-Fi.

**Updates**
- The page inside Revenant updates with the firmware. In Revenant Firmware click Update. It takes seconds over Wi-Fi.

**Why it works this way:** Chrome blocks a secure (https) page from talking to Revenant's plain `http` address. A page that Revenant serves itself shares that address, so nothing is blocked. The copy on woz.lol is only for Bluetooth setup. If you prefer, run `./serve.sh` and open `http://localhost:8090` for a copy on your computer.

## Use the M5 IR module

**The built-in IR on the stick is flaky when it sends and receives at the same time. Both ways need near-perfect aim. Plug in the M5 IR module and it becomes flawless.** In testing, a Zenith Space Command sent to an Apple TV over Bluetooth and to a Sony receiver over IR with near perfect consistency from any aim.

Plug the M5 IR unit into the Grove port before power-up. Its LED (yellow wire, G9) fires together with the built-in one on every send. Its receiver (white wire, G10) listens together with the built-in one, and the stick keeps whichever capture saw more of the frame. The IR In and IR Out rows on the stick say "Both Ready" when the module was found at boot.

## Broadlink

Revenant can send its IR through a Broadlink RM4 mini on your Wi-Fi, so the light can sit right next to the thing it controls. In the Broadlink app open the device and switch on Unlock. In the Broadlink card on the page, press Find Broadlink or type its address, then choose where IR goes out: Revenant, Broadlink, or Both. The choice covers every button, tapped or mapped. The Broadlink uses its own 38 kHz light, which also works for Sony.

## Folders

- `ir_tester/` raw IR tester sketch
- `super_remote/` Revenant firmware sketch
- `web/` the web tool
- `PLAN.md` stages and test order

## Build and update the firmware

`./build.sh` first packs `web/index.html` into the firmware (`super_remote/page_gz.h`, generated and committed), so Revenant can serve it. It then compiles and refreshes the update file in `web/firmware/` (`revenant.bin` and `version.json`). That folder is always committed so anyone can update without a compiler. Bump `FW_VERSION` in `super_remote/super_remote.ino` first.

To update Revenant from the web tool, connect it and use Revenant Firmware, then Update. Over Bluetooth it takes several minutes, over Wi-Fi seconds. The page looks in its own folder and in the online repo, offers whichever version is higher, and prefers its own folder on a tie. A page opened from Revenant itself uses the online repo. Settings and pairings stay.

The first time, or if an update ever fails to start, flash over the cable. Board `m5stack:esp32:m5stack_sticks3`. Hold the power button 3 to 5 seconds for download mode, run `./build.sh flash`, then press the power button once to leave download mode.

## Bluetooth modes

Bluetooth is one thing at a time. In Keyboard mode Revenant is named Revenant Keyboard and only acts as a keyboard, media keys and mouse for a TV. In Setup mode it is named Revenant Setup and only offers the settings link to this page, so a computer never pairs it as a keyboard. A new stick starts in Setup mode. Once it joins Wi-Fi, the page switches it to Keyboard mode by itself. To set up a new network on a stick that cannot reach Wi-Fi, open Revenant menu, Bluetooth, and pick Setup.

One Connect button does the right thing. It tries the saved Wi-Fi address first and falls back to Bluetooth setup only when Revenant is not answering.

## Wi-Fi

Join once over Bluetooth: connect Revenant, type the network name and password in Revenant Wi-Fi, click Join Wi-Fi. When Revenant shows its address the page switches to Wi-Fi by itself, remembers the address, and reconnects on every visit with no Bluetooth chooser. Updates then take seconds. Revenant keeps the network for next time, Forget Wi-Fi clears it. Use http://revenant.local or the local server (`./serve.sh`), a page served over https cannot reach Revenant.

## Security

- Over Bluetooth Revenant ignores everything except HELLO and PING until you press its front button, which opens a two minute window. Someone nearby cannot change settings, send keys or push firmware without touching Revenant.
- Over Wi-Fi every request needs a secret key. Revenant hands the key out only inside that window, over Bluetooth or to the page it serves itself, and the page keeps it. Only the page Revenant serves and the page at http://localhost:8090 may read answers. Forget Wi-Fi retires the key.
- The Wi-Fi password is stored on Revenant in plain form and is never kept by the page. It travels over Bluetooth during setup without link encryption, so set up Wi-Fi when nobody hostile is within Bluetooth range.
- Firmware images are not signed. Anyone with the key can flash Revenant.

## Learning old remotes

Hold the button for about a second while learning. The page collects the frames the remote sends and saves one clean copy, the middle value at each position of the most common length, so a noisy or cut off frame does not spoil it. Matching allows wiggle room, and a cut off press counts when exactly one learned button fits it. The Learning card has three presets (Careful, Balanced, Forgiving) and a Fine Tune section. The settings live on Revenant and apply to both Revenant and the page. Press-by-press matches show in the Live Monitor, with the closest button when nothing matches.

## Revenant buttons

- Front: echo the last heard code. A press also opens the two minute window for the page at http://revenant.local
- Right: menu (Brightness, Screen sleep, Wi-Fi on or off, Frame end, Forget All pairings, Close). Choices are saved
- Power double click: full off
- Screen sleep turns only the screen off. Lifting Revenant or pressing a button wakes it, and the first press only wakes it. Mapping, Wi-Fi and Bluetooth keep running while it sleeps

## Use the web tool

1. Open http://revenant.local, or run `./serve.sh` and open http://localhost:8090 in Chrome.
2. Click Connect Revenant. Over Bluetooth pick Revenant, over Wi-Fi press the front button first.
3. Tap a button to send it. Learn Button listens for a real remote. Layout Mode (the grid icon) drags buttons around. Map Mode (the arrows icon) picks what a remote button should send, and Revenant does it on its own after Save To Stick. Turn on Auto Save To Stick to keep every change on Revenant.

## Pair the Bluetooth keyboard

On the Apple TV open Settings, Remotes And Devices, Bluetooth and pick Revenant Keyboard. The Bluetooth row in the Revenant Wi-Fi card then says Keyboard, paired. Forget All in Revenant menu clears pairings.

## Connect automatically

Chrome can remember Revenant after the first pick, so the page reconnects on its own each time it opens. This needs one setting. Open `chrome://flags/#enable-web-bluetooth-new-permissions-backend`, set it to Enabled, and restart Chrome. Without it you pick Revenant from the list each visit.
