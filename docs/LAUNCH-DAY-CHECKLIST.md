# Launch-day checklist (canards flight computer)

2026-11-21. A short field version of `docs/TEST-PLAN-2026-11-21.md`; the step numbers (2.3 and so on) point there for detail. Never fly `marsv21_debug`.

**The radio is disabled in this build** (`src/main.cpp`, "RADIO DISABLED"): it was the suspect for USB dropping whenever DC power is on. There is no telemetry on the pad. Check the board over USB at the prep table, then rely on the LEDs (green blinking about twice a second = running). Read the flight log by pulling the SD card.

---

## How a passive flight works (no new code)

Power on and **never send `armFlight`**. The board stays in Init, holding the canards at 0° and logging every IMU sample to SD (about 208 rows/s). When it sees a launch (5 g held for 50 ms):

| Time after launch | State | Canards |
|---|---|---|
| 0 s | Boost, `locked` = 1 | powered, held at 0° (locked out: never controlled) |
| 2 s (timeout) | Control | held at 0°; attitude hold is refused (`gates` has 2) |
| 22 s (timeout) | Coast | held at 0° |
| 52 s (timeout) | Chute | servo power off |
| 352 s (timeout) | Land | off |

States change on timeouts, because the estimator only aligns in PreFlight. That doesn't matter: the raw IMU, mag and baro data is all in the log. The watchdog starts at launch. This path is covered by the desktop tests (`test_flight`, "enabled, but no PreFlight") but **has not been run on the board.**

"Held at 0°" means the calibrated centre pulse. The servo PID gains are all zero (`include/VehicleConfig.h`), so the encoders never move the canards. A `CANARDS_NO_ENCODERS` build therefore holds the canards exactly like the normal build. It only loses the `enc0..3` angles in the log and cannot have an encoder interrupt storm.

What a passive flight can't do: you can't replay its log to test the controller, because it never went through PreFlight. That needs the `armPassive` command in the test plan, which doesn't exist yet.

---

## A. Before you leave: get the board flashed

- [ ] **Commit and push `mikayla`.** Write down the hash: the log header does not record it.
  ```bash
  git add -A lib/Actuators src/main.cpp docs && git commit -m "Encoder edge counter and launch checklist" && git push
  git rev-parse --short HEAD
  ```
- [ ] **Canards-Rocket calibration:** commit and push the 2026-11-20 bench data on `feat/canards-state` (the only copy). Check that the servo centres in `include/VehicleConfig.h` match it.
- [ ] **Flash:** `~/.platformio/penv/bin/pio run -e marsv21 -t upload`. If USB fights you, see section F.
- [ ] **Back up, then empty, the SD card.** Log numbers wrap at 100 and overwrite old files; the cards are past log 70.
- [ ] **One boot check, over the radio or USB:**
  - `state` 0, `sensor_ok` 15, `safety` 0
  - `log_rows` climbing about 208/s, `imu_missed` 0, `gap_max_us` about 4800
  - `enc_edges` climbing a few thousand per second
  - Much faster `enc_edges`, or `imu_missed`/`gap_max_us` going bad: **flash the no-encoder build and fly that:**
    ```bash
    PLATFORMIO_BUILD_FLAGS=-DCANARDS_NO_ENCODERS ~/.platformio/penv/bin/pio run -e marsv21 -t upload
    ```
- [ ] **Pack:**
  - ground-station radio
  - charged flight battery and a spare
  - SD card reader for the laptop
  - spare empty SD card
  - USB cable (backup only)
  - a printed copy of this page

## B. Active or passive

**Passive** unless all of these passed on this build, on this airframe, since the canards were last taken apart:
- 2.3 sweep
- 2.4 IMU orientation
- 2.5 arming
- **2.6 control direction**

GNC must also accept the 37 `PLACEHOLDER` values. Steps 2.5 and 2.6 need USB. If USB is unreliable, that alone decides it: **fly passive.**

## C. At the prep table, rocket assembled (USB, no radio)

- [ ] Connect DC first, then USB. Run `.venv/bin/python ground/telemetry_gui.py --usb --flight`.
- [ ] Power on; keep your fingers clear, because the servos drive to 0°. Run the section A boot check.
- [ ] **2.3 Sweep:** film from the tail and click **Actuator test** (Init only); it gets a reply.
  - Order: CH1, CH3, CH4, CH2, each a quarter turn clockwise.
  - Each +5° step turns the leading edge clockwise.
  - About 5° each; all back to 0° afterwards.
- [ ] Unplug USB. Green keeps blinking about twice a second.

## D. On the pad

- [ ] Power on. Green blinks about twice a second and the canards go to 0°. (No telemetry: the radio is off.)
- [ ] **Passive: don't arm. Walk away.**
- [ ] Active flight is not possible in this build: arming needs a command, and with the radio off commands only come over USB.

## E. After recovery

- [ ] **Don't power the board.** Pull the SD card and copy every file to the laptop. The flight is the newest `canardsLog<N>.csv` with a lot of rows.
- [ ] `.venv/bin/python tools/flight_log.py summary <file>`, then hand the log to GNC.

## F. USB not cooperating

Try in this order:
1. **One program on the port at a time.** The GUI, `pio device monitor`, `dumplog` and `usb_stream.py` all fight over it, and on a Mac a second program opening it gets nothing or garbage. Close every other one. `lsof | grep usbmodem` shows who has the port.
2. **Never open the port at 1200 baud.** That reboots the board into DFU, so it "disconnects". The repo's tools all use 115200; some third-party serial monitors don't.
3. **After every flash or reset, the port comes back under a new name.** Restart the GUI rather than reconnecting it.
4. **Use a data cable plugged straight into the laptop, no hub.** Many USB-C cables are charge-only.
5. **Rule out an encoder interrupt storm:** flash the no-encoder build (section A). If USB becomes solid, the cause is an encoder line. That build is fine to fly passive.
6. If the upload can't put the board into DFU itself: hold BOOT0 high, reset the board, and upload again (`scripts/dfu_touch.py` says the same when it fails).
