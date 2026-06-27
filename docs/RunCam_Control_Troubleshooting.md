# RunCam Split 4 — Control Troubleshooting & Diagnostics

**Project:** TRES Titan Avionics
**Symptom:** Pressing Record ON/OFF in the ground station UI does nothing — the camera's
blinking red record LED in the video feed does not change.

---

## 1. The uplink command path is verified GOOD (not the problem)

I traced the entire ground-station → rocket command chain for the camera buttons and it is
correct end-to-end. There is **no hang-up in the ground station's sending of commands**:

1. **UI button** (`_on_cam_toggle` / new toggle buttons) → calls `_send` / `radio.send_frame`.
2. **`protocol.build_standard_frame(stage, cmd)`** builds a 6-byte frame
   `[0x54][0x52][stage][0xC0][cmd][crc8]`, CRC over the first 5 bytes.
3. **`RadioWorker.send_frame`** writes it to the M0 serial port (thread-safe, mutex-guarded).
4. **M0 firmware** transmits it over LoRa.
5. **Teensy `checkUplink()`** validates magic/stage/type, checks `calcCRC8(rxBuf,5)==rxBuf[5]`
   (matches the ground station exactly), and dispatches `CMD_CAM_RECORD_ON/OFF`.
6. **Teensy** calls `runcamStartRecording()` / `runcamStopRecording()` and replies with an ACK.

Because **VTX power/frequency commands work** (you confirmed this), the radio link, framing,
CRC, and Teensy command dispatch are all proven functional — those commands travel the exact
same path. The camera commands reach the Teensy. **The break is strictly between the Teensy
and the camera (the Serial1 UART link or the camera's configuration).**

### How to confirm the command reaches the Teensy
Watch the Teensy USB serial monitor (115200 baud) while pressing the button. You should see:
```
[CMD] 0x10
[CAM] START via power-toggle (RCDP 0x01/0x01)
```
If you see those lines, the entire uplink path works and the problem is 100% Teensy→camera.
If you see `[CMD] 0x10` but the LED never changes, it is the UART wire or camera config.

---

## 2. CONFIRMED ROOT CAUSE: TX/RX wires are reversed

The bench wiring was checked against the PCB silkscreens and it is **wired backwards**. This
is the cause of "the record LED never changes" — commands reach the Teensy and ACK, but the
command-carrying line never reaches the camera's input.

### What the official Split 4 manual specifies (camera-perspective labels)
The RunCam Split 4 silkscreen labels TX/RX **from the camera's own perspective**, so the
manual's Transmitter Connection Diagram requires a **crossover**:

| RunCam pad (silkscreen) | Connects to (FC / Teensy) |
| ----------------------- | ------------------------- |
| **RX** (camera input)   | FC **TX** = Teensy **pin 1** |
| **TX** (camera output)  | FC **RX** = Teensy **pin 0** |
| **GND**                 | Teensy **GND**              |

### What was actually wired (incorrect — straight-through)
| RunCam pad | Was wired to | Result |
| ---------- | ------------ | ------ |
| **RX** (camera input)  | Teensy **pin 0** (Teensy RX) | ❌ camera input tied to Teensy input |
| **TX** (camera output) | Teensy **pin 1** (Teensy TX) | ❌ two outputs tied together |

Because the Teensy's TX (pin 1) — the line that carries record/Wi-Fi commands — is connected to
the camera's TX (an output) instead of the camera's RX (its input), **the camera never receives
a single command byte.** It can never respond, and the record LED never changes. This matches
the symptom exactly.

### The fix — swap the two signal wires
- Teensy **pin 1 (TX)** → RunCam **RX**
- Teensy **pin 0 (RX)** → RunCam **TX**
- GND ↔ GND (unchanged)

This is the crossover the **firmware already assumes** (`pin 1 = Teensy TX → RunCam RX,
pin 0 = Teensy RX ← RunCam TX`). The code was correct; only the physical harness was reversed.
After swapping, the `⏺ Toggle` button should flip the record LED.

### Can't re-solder right now? Firmware workaround is already in place
If the wires cannot be physically corrected yet, the firmware now ships a compile-time switch
**`CAM_WIRING_SWAPPED = 1`** (top of `Teensy_TRES_v7.ino`, RunCam UART section). When set to 1,
the Teensy drives the camera over a bit-banged **SoftwareSerial** port with **TX on pin 0** and
**RX on pin 1** — i.e. it transmits commands on the pin that is physically wired to the camera's
RX, so control works with the reversed harness and **no re-soldering**. Requirements/notes:

- Needs **Teensyduino ≥ 1.60** (full SoftwareSerial RX+TX on Teensy 4.x). Your build already
  uses the 1.60 hardware package, so this is satisfied.
- Only the Teensy→camera (TX, pin 0) direction is required for record/Wi-Fi/toggle control.
  The camera→Teensy reply (RX, pin 1) is best-effort; if it is flaky at 115200, `FAULT_CAM` may
  stay set but control still works (`camUsePowerToggle` defaults safe).
- This is half-duplex and camera traffic is infrequent, so it does not meaningfully affect LoRa
  or OSD timing.
- **Once you do correct the wiring to the proper crossover, set `CAM_WIRING_SWAPPED` back to 0**
  to return to efficient hardware `Serial1`.


Also confirm a **common ground** between the Teensy and the camera — the Split takes ground
from the power supply, and without a shared ground the UART idles at random levels. Only the
Teensy-TX → camera-RX line (plus shared ground) is strictly required for control; the
camera-TX → Teensy-RX line is only needed for the camera's device-info reply (the `FAULT_CAM`
clear).

---

## 3. Use the new diagnostic: "⏺ Toggle" button

I added a **`CMD_CAM_TOGGLE` (0x14)** command and an **"S1/S2 ⏺ Toggle"** button to the RunCam
panel. Unlike Record ON/OFF, it sends **one unconditional power-button press** (RCDEVICE action
`0x01`) every time, regardless of what the firmware believes the recording state is.

This is the cleanest Teensy→camera link test:

- **LED changes when you press ⏺ Toggle** → the UART link and protocol are good. Any prior
  "nothing happens" was a state-desync in the ON/OFF logic, now bypassed. Use ⏺ Toggle as your
  primary record control.
- **LED still does not change** → the problem is physical (wiring per §2) or camera config
  (§4). The bytes are leaving the Teensy correctly; the camera isn't acting on them.

Why ON/OFF could appear dead even with good wiring: the toggle relies on the believed
`camRecording` flag. If the camera auto-records on boot (common), the Teensy boots thinking
"idle" while the camera is actually recording — so the first "Record ON" sends a toggle that
*stops* it, and the LED change is easy to miss, or subsequent presses cancel out. The
unconditional ⏺ Toggle removes that ambiguity.

---

## 4. Camera configuration checklist (RunCam app)

For the power-button toggle to control recording, the camera must be set up correctly. These
are one-time settings via the RunCam phone app (or the on-camera menu):

- **Default mode = Video** (not Photo). A power-button press only toggles *recording* in video
  mode. In photo mode it takes a still and the LED behaves differently.
- **Auto-record on power-up**: know whether it is ON or OFF. If ON, the camera is already
  recording at boot — factor that into what the LED is telling you.
- **Loop Recording = OFF** (see `RunCam_Split4_Findings.md`) — unrelated to control, but this
  is what split your Stage 2 launch into a new file.
- **Camera fully booted before commands**: the Split 4 takes 2–3 s (sometimes more) to boot.
  `initRunCam()` waits 2 s and retries; if the camera is slow, the `FAULT_CAM` flag is set and
  the background re-probe (every 30 s) clears it once the camera answers.

### Check the CAM fault flag in the ground station
The STATUS packet carries `fault_flags`. If the **CAM** fault bit (0x40) is **set**, the Teensy
sent the device-info query and **got no reply** — that means the camera's TX→Teensy RX line
(or the whole link) is dead, which usually means the wires are swapped or there is no common
ground. A clear CAM flag means the camera answered at least once and the link is bidirectional.

> Note: control only needs Teensy TX → camera RX. It is possible for control to work while the
> CAM fault stays set (if only the camera→Teensy reply line is broken). So use the LED + the
> ⏺ Toggle button as the definitive control test, and the CAM flag as a link-health indicator.

---

## 5. Wi-Fi vs. OSD menu — how to configure the camera remotely

### Verified: the Wi-Fi command is real but does little on a recording Split 4
Bench result: **record (power button, 0x01) works, Wi-Fi (0x00) shows no LED change.** That is
expected, and the earlier "wire it up and use Betaflight Camera Wi-Fi" advice was generic and
misleading for this case. Verified against primary sources:

- RunCam Wi-Fi requires the **Wi-Fi module/capability**, and the toggle only fires while the
  camera is in **standby** — not in video/recording mode. Since record works, the camera is in
  video mode, so a Wi-Fi-button press there does not toggle Wi-Fi (no LED1 flash).
- RunCam's protocol notes state `SIMULATE_WIFI_BTN` on a Split "simulates the mode button… you
  can only navigate, not change a setting." So even when it does something, it is not a path to
  changing settings.

The `📶 Wi-Fi` buttons are kept (harmless, and they work if your unit has the Wi-Fi module and
is in standby), but **Wi-Fi is not the way to configure this camera in the field.**

### Use the on-screen OSD menu instead (no Wi-Fi, no physical button)
The Split 4's settings live in its **OSD menu**, drawn over the analog video feed you already
see in the ground station. Per the Split 4 manual, the protocol drives it like this:

- **`⚙ OSD Mode` button → `CMD_CAM_CHANGE_MODE` (RCDEVICE 0x02):** switch between Video and OSD
  setup mode; when already in the OSD, this **exits** the menu.
- **`⏺ Toggle` button → `CMD_CAM_TOGGLE` (RCDEVICE 0x01, power):** in the OSD it **moves to the
  next menu item**; in video mode it starts/stops recording.

So to change a setting (e.g. turn **Loop Recording OFF**): press **⚙ OSD Mode** to bring up the
menu on the video feed, use **⏺ Toggle** to step through items, and **⚙ OSD Mode** again to
exit. Everything is visible on the video panel — no Wi-Fi and no working button required.

> Caveat: this is a 2-button on-screen menu, so navigation is sequential and a little tedious,
> and the exact item/value behavior depends on the camera's firmware menu. Test it on the bench
> with the video feed up before relying on it. If your unit turns out to have a real Wi-Fi
> module, you can still use `📶 Wi-Fi` (camera in standby) to pair the phone app for a richer UI.

### Original Wi-Fi notes (kept for reference)


You mentioned the physical Wi-Fi button broke off. I added **`CMD_CAM_WIFI` (0x13)** and
**"S1/S2 📶 Wi-Fi"** buttons to the RunCam panel. This sends RCDEVICE action `0x00` (simulate
Wi-Fi button), which toggles the camera's Wi-Fi exactly like the physical button — so you can
pair the RunCam app to change settings (Loop Recording, resolution, etc.) without the button.

Important caveats:
- **Ground use only.** On the Split 4, enabling Wi-Fi typically **suspends recording**. Never
  trigger Wi-Fi in flight.
- Wi-Fi toggle goes over the same Teensy TX → camera RX line, so it is also a valid link test:
  if the camera's Wi-Fi LED/AP appears when you press 📶 Wi-Fi, the control link works.
- This only replaces the *button*. To actually connect, you still join the camera's Wi-Fi AP
  from your phone and use the RunCam app.

---

## 6. Recommended bring-up sequence

1. Flash the updated Teensy firmware and ground station.
2. Open the Teensy serial monitor (115200). Power the camera; confirm `initRunCam()` reports
   the camera live (or watch for the 30 s re-probe clearing `FAULT_CAM`).
3. Press **S1 ⏺ Toggle**. Watch the record LED in the video feed.
   - **LED toggles** → done. Link + protocol confirmed. Use ⏺ Toggle (or ON/OFF) normally.
   - **No change** → swap the Serial1 TX/RX wires (§2), confirm common ground, retest.
4. Press **S1 📶 Wi-Fi** on the ground; confirm the camera's Wi-Fi comes up, then pair the app
   and set Default Mode = Video, Loop Recording = OFF.
5. Re-test record control after the camera is confirmed in video mode.

---

## Sources

- [RunCam Split 4 / 4K official manual — Transmitter Connection Diagram (Camera RX→FC TX, Camera TX→FC RX)](https://www.runcam.com/download/split4k/RC_Split_4k_Manual_EN.pdf)
- [RunCam Device Protocol — official spec (actions 0x00 Wi-Fi / 0x01 power)](https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol)
- [ArduPilot RunCam support — wiring note "try reversing the signal wires", Split 4 quirks](https://ardupilot.org/plane/docs/common-camera-runcam.html)
- [Betaflight `rcdevice.c` — camera-control packet build / power-button path](https://github.com/betaflight/betaflight/blob/master/src/main/io/rcdevice.c)
