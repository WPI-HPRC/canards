# Flight log, SD replay and USB stream tools

Python 3. `flight_log.py` needs only the standard library; `usb_stream.py`
also needs pyserial (`pip install -r tools/requirements.txt`).

**SD replay and log streaming are OPEN LOOP.** The recorded motion was caused
by the commands of the code that flew, so the new code's commands change
nothing that happens next. They show whether the new code changes state at the
same moments and commands sane values. They do NOT show whether it would have
flown straight. Closing the loop needs the RocketPy sim (Voyager-launch-controller)
driving `usb_stream.py`'s row/reply interface: the sim sends one step of sensor
data, the board runs one cycle and replies with its commands, the sim advances.
That is future work: the sim reads attitude from truth and has no accelerometer,
barometer or magnetometer model yet (docs/HANDOFF-flight-skeleton-2026-11-20.md,
section 7).

## The log

One CSV row per cycle (`src/Subsystems/Logger/CycleLogFormat.h`), with a
header. The input columns (`t_us` .. `cmds`) are written losslessly, so a
replay reproduces the original bit for bit; the output columns (state, canard
mode, commands, pulses, safety bits) follow. Readers find columns by name.
`testdata/sample_log.csv` is a short scripted flight recorded by the desktop
tests (`SAMPLE_LOG_OUT=$PWD/tools/testdata/sample_log.csv pio test -e native -f test_log_replay`).

## Bench without the ground radio (USB bench console)

In Flight mode (env `marsv21`), while a host has the board's USB port open,
the board also sends its telemetry rows over USB (the radio's `$H` / `$D` /
`$A` lines, without the radio framing) and takes commands over USB, one per
line:

```
armFlight          same as the GUI button: PreFlight + enable (starts the watchdog)
actuatorTest       the canard sweep, Init only
list | dump [N]    Nic's log download, Init only and only before arming
```

The commands follow the same rules as over the radio. To use the ground GUI on
USB:

```bash
python3 ground/telemetry_gui.py --usb --flight    # pick the board's port, Connect
```

With USB unplugged, as in flight, the console writes nothing. It never blocks
the loop: a row that can't go out yet is skipped. Open the port at 115200,
**never 1200** (that reboots into DFU). A host that has the port open during
boot makes the board wait 1.5 s for a mode line before it starts Flight.

## Record and download

Every boot of the flight program writes `/logs/canardsLog<N>.csv` on the SD
card. Download over USB with Nic's targets (board in Init, flight firmware):

```bash
~/.platformio/penv/bin/pio run -e marsv21 -t listlogs              # what's on the card
~/.platformio/penv/bin/pio run -e marsv21 -t dumplog               # the previous boot's log
LOG_INDEX=7 ~/.platformio/penv/bin/pio run -e marsv21 -t dumplog   # /logs/canardsLog7.csv
```

## Read and compare

```bash
python3 tools/flight_log.py summary LOG.csv        # states + entry times, canard modes, safety bits, drops
python3 tools/flight_log.py compare A.csv B.csv    # first row where state/mode/cmd0..3/pulse0..3 differ
python3 tools/flight_log.py inputs LOG.csv -o IN.csv   # only the input columns
```

`compare` aligns rows by `t_us` and exits 1 if anything differs.

## Boot modes

The flight program boots in **Flight** mode unless a USB host has the board's
port open at the end of `setup()`. Then the board waits up to 1.5 s for one
line:

```
mode replay <N> [servos] [fast]
mode stream [servos]
```

Anything else, or nothing, is Flight. With no host connected the board never
waits. Replay and stream are never flight-ready: no watchdog, no log download,
no ground-station commands; telemetry rows still go out with `sim_mode` = 1
(replay) or 2 (stream). The servos move only with `servos`.

`usb_stream.py` handles the timing: it asks you to reset the board, waits for
the USB port to go away and come back, opens it at once and sends the mode
line. It always opens the port at 115200 baud: **never open it at 1200 baud**,
that reboots the board into its DFU bootloader.

## Replay a log on the board

```bash
python3 tools/usb_stream.py replay 7              # paced by the recorded sample times
python3 tools/usb_stream.py replay 7 --fast       # as fast as the SD log keeps up
python3 tools/usb_stream.py replay 7 --servos     # and let the servos move (OPEN LOOP)
python3 tools/usb_stream.py replay 7 --servos --detach   # let go of the port once it starts, so
                                                         # ground/telemetry_gui.py --usb --flight can watch
```

(Or type `mode replay 7` into a serial terminal within 1.5 s of the board's
boot messages.) The board replays `/logs/canardsLog7.csv` through the real
flight program and writes its own new log, this boot's file (the banner names
it). When it ends it prints `# replay done: ...`. Then reboot normally,
download both logs (into `logs/`, named with a time stamp), and compare:

```bash
LOG_INDEX=7 ~/.platformio/penv/bin/pio run -e marsv21 -t dumplog    # the original
LOG_INDEX=8 ~/.platformio/penv/bin/pio run -e marsv21 -t dumplog    # the replay (the index from the banner)
python3 tools/flight_log.py compare logs/canardsLog7_<stamp>.csv logs/canardsLog8_<stamp>.csv
```

The same code on the same log gives no differences. A difference shows where
the new code first decides something else.

## Stream rows over USB

```bash
python3 tools/usb_stream.py stream LOG.csv -o replies.csv     # a recorded log, as it is
python3 tools/usb_stream.py stream --synthetic                # a generated flight
python3 tools/usb_stream.py stream --synthetic --roll-rate 20 --servos -o sign.csv
                                       # control-sign check: in Control, cmd0..3 must be NEGATIVE
                                       # (the canards oppose a +roll, clockwise seen from the tail)
python3 tools/usb_stream.py synthetic > synth.csv             # just print the generated rows
```

Protocol (`src/Subsystems/Logger/StreamProtocol.h`), one line each way, in
lockstep: the host sends the log header once (the board replies with its own
log header), then one input row per step in the log row format; the board runs
one cycle per row and replies with that cycle's full log row, so the host can
read `cmd0..3` / `pulse0..3`. Lines starting with `#` are comments (no reply).
A bad line gets `!ERR <reason>`. The board's boot messages end with
`# stream ready`; after it the board sends only replies. With `servos`, if no
line arrives for 500 ms the board puts the canards at 0 degrees until lines
resume.

The synthetic flight: 5 s on the pad (EnterPreFlight + CanardsEnable at 1 s),
a 1.5 s 7 g burn, a coast with drag to apogee (about 370 m), an 8 m/s descent,
3 s on the ground. It runs through the board's real estimator, so what the
states do depends on that estimator.
