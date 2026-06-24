# RunCam Split 4 — Recording Control & Video Length Findings

**Project:** TRES Titan Avionics
**Date:** 2026-06-23
**Scope:** Why competition video failed, whether the Teensy can control record on/off and clip length, and exactly what (if anything) must be set in an app/Betaflight.

---

## TL;DR

1. **Yes — the Teensy can start/stop recording over UART.** The wiring (Serial1, pins 0/1) and the RunCam Device Protocol (RCDEVICE) frame your code sends are correct. But the Split 4 has a **known firmware bug** that makes it ignore the *explicit* start/stop commands your code currently uses (actions `0x03`/`0x04`). You must switch to the **power-button toggle** (action `0x01`) instead. This is almost certainly why Stage 1 produced empty files.

2. **No — the Teensy cannot set clip length.** Video file/segment length is **not** part of the RCDEVICE protocol. There is no UART command for it. It is controlled entirely on the camera itself via the **RunCam mobile app** (Wi‑Fi), *not* Betaflight. Betaflight only does record on/off and OSD — it has no "video length" setting either.

3. **The Stage 2 "new file at 50 ft" failure was Loop Recording.** With Loop Recording **on**, the Split 4 chops video into fixed time segments (e.g. 1/3/5 min) and, when the card fills, overwrites the oldest. Turn Loop Recording **off** in the RunCam app and each recording becomes one continuous file, auto-split only at the 4 GB filesystem limit (~18 min at 30 Mbps, ~45 min at 12 Mbps). That gets you well past 15 minutes without any code change.

---

## Part 1 — Can the Teensy control record on/off? (Yes, with a one-line fix)

### What your code does today (correct parts)

Your firmware already implements the RunCam Device Protocol correctly in these respects:

- **Port:** `Serial1` on pins 0 (RX) / 1 (TX) at **115200 baud** — correct. RCDEVICE always runs at 115200.
- **Frame format:** `[0xCC][CmdID][Action][CRC8]`, 4 bytes for camera control — matches the official spec exactly.
- **CRC:** CRC‑8/DVB‑S2 (poly `0xD5`) computed over the whole frame *including* the `0xCC` header. I verified this byte-for-byte against Betaflight's own `runcamDeviceSendPacket()`. Your `crc8_dvbs2(frame, rcdpCrcLen)` call is right. (Note this is the opposite of your VTX/SmartAudio path, which correctly skips the sync byte — easy to confuse, but both are currently correct.)
- **Command plumbing:** `CMD_CAM_RECORD_ON` → `runcamStartRecording()` → frame on Serial1, and the ACK path, are all wired correctly end to end.

### The actual bug

Your `runcamStartRecording()` / `runcamStopRecording()` send:

```c
#define RCDP_ACT_START 0x03   // Explicit start recording (v2)
#define RCDP_ACT_STOP  0x04   // Explicit stop  recording (v2)
```

These are the *explicit* "start recording" / "stop recording" actions in the RCDEVICE spec. The problem: **the RunCam Split 4 (and Split 4 4K) firmware does not properly implement `0x03`/`0x04`.** This is a documented, well-known defect — ArduPilot ships a dedicated workaround for exactly this camera (`CAM1_RC_TYPE = 3`, "RunCam Split4 4k"), precisely because the explicit recording commands don't work and it has to fall back to **power-button simulation**.

Your own code even anticipates this in a comment ("If the RunCam does not respond to 0x03/0x04 … change to action 0x01 SIMULATE_POWER_BTN = toggle") — that fallback is the one you actually need.

> **What the camera *does* obey:** action `0x01` = **simulate power button**. On the Split 4 in **video mode**, a power-button press toggles recording start ↔ stop, exactly like physically tapping the button. This is how Betaflight's "CAMERA POWER" switch starts/stops Split recordings.

### The fix (firmware)

Change the two action constants to use the power-button toggle, and treat start/stop as a **toggle of known state** rather than two distinct idempotent commands.

```c
// Split 4 ignores explicit 0x03/0x04. Use power-button toggle (0x01).
#define RCDP_ACT_POWER  0x01   // Simulate power button = toggle record in video mode

void runcamStartRecording() {
  if (camRecording) return;                 // already recording — don't toggle OFF
  uint8_t action = RCDP_ACT_POWER;          // 0x01 toggle
  runcamSendCmd(RCDP_CMD_CTRL, &action, 1);
  camRecording = true;
  Serial.println(F("[CAM] Power-toggle START sent (RCDP 0x01)"));
}

void runcamStopRecording() {
  if (!camRecording) return;                // already stopped — don't toggle ON
  uint8_t action = RCDP_ACT_POWER;          // 0x01 toggle
  runcamSendCmd(RCDP_CMD_CTRL, &action, 1);
  camRecording = false;
  Serial.println(F("[CAM] Power-toggle STOP sent (RCDP 0x01)"));
}
```

**Why the guard matters:** because `0x01` is a *toggle*, the `camRecording` flag MUST always reflect the real camera state. If the firmware thinks it's recording but the camera isn't (or vice versa), every future command is inverted. So:

- Guard both functions with the `camRecording` flag (shown above) so repeated `RECORD_ON` commands don't accidentally stop a running recording.
- Make sure the camera **boots in video mode**, not photo mode, so the power-button toggle controls *recording* (set once in the app — see Part 3).

### Recommended robustness upgrades (optional but worth it for a one-shot flight)

1. **Read the device-info response and check the feature bits.** At init you already send "get device info" (`0xCC 0x00 …`) but you flush and discard the reply. Parse it: the response is `[0xCC][protocolVer][featureLow][featureHigh][CRC]`. Bits 6/7 = start/stop recording support; bit 0 = power-button support. If bits 6/7 are clear (expected on Split 4), you *know* to use the toggle. This converts a guess into a verified decision.

2. **Send the toggle twice with confirmation, or add a visual confirmation.** Because there's no recording-state feedback on the wire, consider lighting an LED / OSD flag from `camRecording`, and on the pad have a human confirm the Split 4's onboard record indicator before launch.

3. **Don't auto-stop on mode changes during flight.** Right now `setGroundMode()` calls `runcamStopRecording()` on transitions to TEST/PAD. Double-check the in-flight state machine can never call that mid-boost, or a stray toggle will stop your launch footage. (This is a plausible secondary cause of the Stage 1 empty file: a start toggle followed by an unintended second toggle = net "not recording.")

---

## Part 2 — Can the Teensy set video clip length? (No — not over UART)

**There is no RCDEVICE command for recording length, segment time, bitrate, or resolution.** The protocol only exposes: get device info, camera control (power/Wi‑Fi/mode/start/stop), 5‑key OSD menu simulation, and FC attitude. Clip length is a *camera setting*, stored on the camera, and the only ways to change it are:

- **The RunCam mobile app over Wi‑Fi** (primary method for the Split 4), or
- The on-camera **5‑key OSD menu** — but the Split 4 has no analog OSD menu for this; it's app-driven.

So the Teensy can decide *when* recording starts and stops, but not *how the camera chooses to chunk the file.* That's why Stage 2 split at 50 ft regardless of your code: the camera's **Loop Recording** setting decided that, independent of the UART.

### What actually controls file length on a Split 4

| Setting | Where | Effect |
|---|---|---|
| **Loop Recording = OFF** | RunCam app → Settings → Video | One continuous file per recording session. Auto-splits **only** at 4 GB. |
| **Loop Recording = ON** | RunCam app → Settings → Video | File chopped into fixed segments (commonly 1 / 3 / 5 min). Oldest overwritten when card fills. **This is what bit you on Stage 2.** |
| **4 GB filesystem cap** | Firmware (fixed) | Hard split at 4 GB even with loop off. ≈18 min @ 30 Mbps, ≈45 min @ 12 Mbps. |

**Bottom line on "15-minute files":** You can't dial in "15:00" exactly. But turning **Loop Recording OFF** gives you a single continuous file that only splits at 4 GB — comfortably longer than 15 minutes at any sane bitrate. If you specifically want guaranteed >15 min in one file, run a **bitrate of ≤30 Mbps** (≈18 min/4 GB) or lower (12 Mbps ≈ 45 min/4 GB). Even if it splits at 4 GB, the two clips are gap-free and trivially re-joined; you will not lose the launch the way loop-segmenting did.

---

## Part 3 — Camera setup guide (RunCam app, NOT Betaflight)

> **You do not need Betaflight at all** for this rocket. Betaflight is flight-controller firmware; your flight computer is the Teensy. Betaflight's only relevance to a RunCam is (a) record on/off over UART — which your Teensy already does — and (b) OSD. It has **no** video-length setting. Everything below is done in the RunCam phone app.

### One-time camera configuration (do this on the bench, before flight)

1. Power the Split 4 standalone (or on the rocket with the lens module + Wi‑Fi module attached).
2. Press the **Wi‑Fi button** on the camera (or send RCDEVICE action `0x00` to toggle Wi‑Fi). The camera broadcasts a Wi‑Fi AP.
3. On your phone, install **RunCam App** (iOS App Store / Android APK from runcam.com), join the camera's Wi‑Fi network, open the app.
4. Go to **Settings → Video** and set:
   - **Loop Recording → OFF**  ← *the critical fix for the Stage 2 problem.*
   - **Resolution / Frame rate →** your choice (e.g. 1080p60 or 2.7K). Higher res ⇒ higher bitrate ⇒ shorter time-to-4GB.
   - **Bitrate →** if available, choose ≤30 Mbps to guarantee >15 min before the 4 GB split (12 Mbps ≈ 45 min).
5. Confirm the camera's **default mode is Video** (not Photo), so a power-button toggle controls recording. The mode persists across power cycles.
6. **Format the microSD card in-camera** (app → Settings → Format) before every flight. Use a high-endurance card; corrupt/empty files are frequently a card or power-stability symptom — see below.
7. Power-cycle and verify the setting stuck.

### Verify the Teensy → camera control on the bench

1. Flash the firmware with the **`0x01` power-toggle fix** from Part 1.
2. With the camera in video mode and idle, send `CMD_CAM_RECORD_ON` from the ground station. The camera's record indicator should start; a new file should appear on the card.
3. Send `CMD_CAM_RECORD_OFF`. Recording stops; file finalizes.
4. Pull the card and confirm a **playable, non-empty** file. Repeat 3–4 times to confirm the toggle stays in sync with `camRecording`.

---

## Part 4 — Why the files were empty/corrupt (root-cause checklist)

Empty/zero-byte video files on a Split 4 almost always come from one of these. In priority order for your case:

1. **Recording never actually started** — the explicit `0x03` command was ignored (Part 1). The camera created/opened a file handle context but never wrote frames, or the file was opened then immediately closed by a second stray toggle. **→ Fix: action `0x01` + state guard.**
2. **Power instability / brownout at boot or motor ignition.** The Split 4 is sensitive to voltage dips; a brownout mid-write leaves a 0-byte or unfinalized file (no moov atom). **→ Fix: clean, adequately-sized 5 V supply to the camera, decoupling caps, and ideally don't share a rail that sags at motor ignition.**
3. **microSD not formatted in-camera / too slow / failing.** **→ Fix: in-camera format, high-endurance UHS card.**
4. **Loop Recording overwrote/segmented the launch** (the Stage 2 symptom specifically). **→ Fix: Loop Recording OFF.**
5. **Mode-change auto-stop firing mid-flight** (see Part 1, item 3). **→ Audit the state machine.**

I recommend addressing #1, #2, and #4 before the next flight — they're cheap and cover the most likely causes.

---

## Summary table

| Question | Answer |
|---|---|
| Can the Teensy start/stop recording over UART (pins 0/1)? | **Yes**, after switching from action `0x03/0x04` to power-toggle `0x01`. |
| Can the Teensy set video length over UART? | **No.** Not in the RCDEVICE protocol. |
| Is Betaflight required? | **No.** It has no length setting and you don't run it on this rocket. |
| Where is video length controlled? | **RunCam phone app** → Settings → Video (Loop Recording + bitrate). |
| Can I get 15-min files? | Effectively yes: **Loop Recording OFF** → single file, splits only at 4 GB (~18 min @30 Mbps, ~45 min @12 Mbps). |
| Most likely cause of Stage 1 empty files? | Explicit start command ignored by Split 4 firmware (+ possible power brownout). |
| Cause of Stage 2 split at 50 ft? | **Loop Recording** segment time on the camera, unrelated to the Teensy. |

---

## Sources

- [RunCam Device Protocol — official spec](https://support.runcam.com/hc/en-us/articles/360014537794-RunCam-Device-Protocol)
- [Betaflight `rcdevice.h` — command/feature/action constants](https://github.com/betaflight/betaflight/blob/master/src/main/io/rcdevice.h)
- [Betaflight `rcdevice.c` — frame build, CRC over full packet, power-button path](https://github.com/betaflight/betaflight/blob/master/src/main/io/rcdevice.c)
- [ArduPilot RunCam camera support — Split 4 4K start/stop bug, CAM_RC_TYPE=3 workaround](https://ardupilot.org/plane/docs/common-camera-runcam.html)
- [ArduPilot `AP_RunCam.cpp` — Split-series control implementation](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_Camera/AP_RunCam.cpp)
- [RunCam FAQ — "Will RunCam Split save video by segment automatically?" (loop recording + 4 GB split, bitrate→length math)](https://support.runcam.com/hc/en-us/articles/115012639607-Will-RunCam-Split-save-video-by-segment-automatically)
- [RunCam Split 4K manual (UART wiring, app settings)](https://www.runcam.com/download/split4k/RC_Split_4k_Manual_EN.pdf)
- [ArduPilot Discourse — "Control RunCam Split 4 video recording from a radio transmitter"](https://discuss.ardupilot.org/t/control-runcam-split-4-video-recording-from-a-radio-transmitter/62439)
