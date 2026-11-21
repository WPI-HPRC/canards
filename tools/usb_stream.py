#!/usr/bin/env python3
"""Stream sensor rows into the flight program on the board over USB, or start
an SD replay on it. The board side is src/main.cpp (boot modes) and
src/Subsystems/Logger/StreamProtocol.h (the protocol).

  python3 tools/usb_stream.py stream LOG.csv [--servos] [-o REPLIES.csv]
  python3 tools/usb_stream.py stream --synthetic [--servos] [-o REPLIES.csv]
  python3 tools/usb_stream.py replay N [--servos] [--fast]

HOW THE BOARD IS PUT IN A MODE: at the end of its setup(), if a USB host has
the port open, the board waits up to 1.5 s for one line, "mode stream
[servos]" or "mode replay <N> [servos] [fast]". With no host, or no line, it
boots the normal flight program. Opening the port of a board that is already
running does nothing, so by default this tool waits for a FRESH enumeration:
it asks you to reset the board (RESET button or replug), waits for the port to
go away and come back, opens it at once and sends the mode line. With
--no-reset it opens the port as it is now (only useful if the board is still
in its boot window).

The port is always opened at 115200 baud, NEVER 1200: opening it at 1200 baud
reboots the board into its DFU bootloader (src/dfu_bootloader.cpp).

The stream runs in lockstep: send one row, wait for its reply (one line: the
board's full log row for that cycle, or "!ERR <reason>"), then the next. The
replies are saved as a CSV log that tools/flight_log.py can read and compare.

THIS IS OPEN LOOP: a recorded log (or the synthetic flight) does not react to
the board's commands. It shows whether the code changes state at the same
moments and commands sane values, not whether the rocket would fly straight.
Closing the loop needs the RocketPy sim (Voyager-launch-controller) driving
this row/reply interface; that is future work (the sim has no accelerometer,
barometer or magnetometer model yet).

Needs pyserial (tools/requirements.txt).
"""

import argparse
import math
import os
import sys
import time

STM_VID = 0x0483
STM_CDC_PID = 0x5740
BAUD = 115200          # anything but 1200 (1200 = reboot into DFU)
READY_LINE = "# stream ready"
READY_TIMEOUT_S = 20   # setup() mounts the SD card and configures the radio first
ENUM_TIMEOUT_S = 60

INPUT_HEADER = ("t_us,imu_ok,ax,ay,az,gx,gy,gz,mag_new,mx,my,mz,baro_new,p_pa,temp_c,"
                "enc_ok,enc0,enc1,enc2,enc3,cmds")

G = 9.80665


# --- The board's port ---------------------------------------------------------------

def find_port():
    from serial.tools import list_ports
    for p in list_ports.comports():
        if p.vid == STM_VID and p.pid == STM_CDC_PID:
            return p.device
    return None


def port_present(port):
    if port is None:
        return find_port()
    from serial.tools import list_ports
    return port if any(p.device == port for p in list_ports.comports()) else None


def wait_until(cond, timeout_s, what):
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        v = cond()
        if v:
            return v
        time.sleep(0.02)
    raise SystemExit("timed out waiting for %s" % what)


def open_port(port, timeout_s=2.0):
    import serial
    end = time.monotonic() + timeout_s
    while True:
        try:
            s = serial.Serial(port, BAUD, timeout=0.1)  # opens with DTR set
            return s
        except (OSError, serial.SerialException):
            if time.monotonic() > end:
                raise
            time.sleep(0.02)


def connect(port, mode_line, no_reset):
    """Opens the board's port in its boot window and sends `mode_line`."""
    if no_reset:
        dev = port or find_port()
        if not dev:
            raise SystemExit("no board found (USB %04x:%04x); use --port" % (STM_VID, STM_CDC_PID))
    else:
        if port_present(port):
            print("Reset the board now (RESET button, or unplug and replug USB)...", file=sys.stderr)
            wait_until(lambda: not port_present(port), ENUM_TIMEOUT_S, "the board to reset")
        else:
            print("Waiting for the board to enumerate (plug it in or reset it)...", file=sys.stderr)
        dev = wait_until(lambda: port_present(port), ENUM_TIMEOUT_S, "the board's USB port")
    ser = open_port(dev)
    ser.write((mode_line + "\n").encode())
    ser.flush()
    print("%s: sent '%s'" % (dev, mode_line), file=sys.stderr)
    return ser


def read_line(ser, timeout_s, buf):
    """One line (without the newline), or None on timeout. `buf` keeps bytes
    between calls."""
    end = time.monotonic() + timeout_s
    while True:
        i = buf.find(b"\n")
        if i >= 0:
            line = bytes(buf[:i])
            del buf[:i + 1]
            return line.decode(errors="replace").rstrip("\r")
        if time.monotonic() > end:
            return None
        chunk = ser.read(ser.in_waiting or 1)
        if chunk:
            buf.extend(chunk)


# --- Input rows -----------------------------------------------------------------------

def log_lines(path):
    """The header and rows of a recorded log, as they are (output columns
    included: the board ignores them)."""
    with open(path) as f:
        for line in f:
            line = line.rstrip("\r\n")
            if line.strip() and not line.lstrip().startswith("#"):
                yield line


def pressure_at(alt_m):
    return 101325.0 * (1.0 - 2.25577e-5 * alt_m) ** 5.25588


def synthetic_lines(rate_hz=200.0, pad_s=5.0, burn_s=1.5, burn_g=7.0, coast_drag_g=0.3, descent_mps=8.0,
                    roll_rate_dps=0.0):
    """A short scripted vertical flight, in the input row format: on the pad
    (arm commands at 1 s), a 7 g (over the 5 g launch threshold) burn,
    a coast with drag up to apogee, a steady descent under the chute, then
    on the ground. Sensor axes: the IMU's +z is the nose
    (include/VehicleConfig.h kImuToBody). No noise: easy to reason about.

    roll_rate_dps: a constant roll rate about the nose during burn and coast
    (positive = clockwise seen from the tail = +roll). For the control-sign
    check: while the board is in Control the canards must OPPOSE it, so a
    positive rate must give negative cmd0..3 (leading edges anticlockwise
    seen from the tail)."""
    roll_rate = math.radians(roll_rate_dps)
    yield INPUT_HEADER
    dt = 1.0 / rate_hz
    t0_us = 1000000
    period_us = int(round(1e6 / rate_hz))
    h, v = 0.0, 0.0
    phase = "pad"
    landed_at = None
    k = 0
    while True:
        t = k * dt
        if phase == "pad" and t >= pad_s:
            phase = "burn"
        if phase == "burn" and t >= pad_s + burn_s:
            phase = "coast"
        if phase == "coast" and v <= 0.0:
            phase = "descent"
        if phase == "descent" and h <= 0.0:
            phase, h, v, landed_at = "ground", 0.0, 0.0, t

        # Specific force along the nose (what the accelerometer reads), m/s^2.
        if phase == "burn":
            sf = burn_g * G
        elif phase == "coast":
            sf = -coast_drag_g * G
        else:
            sf = G  # at rest, or steady descent (drag balances gravity)

        cmds = "0:-1:0|2:-1:0" if k == int(1.0 * rate_hz) else ""  # EnterPreFlight, CanardsEnable
        mag_new = 1 if k % 2 == 0 else 0
        baro_new = 1 if k % 2 == 0 else 0
        row = [
            str(t0_us + k * period_us), "1",
            "0", "0", "%.9g" % sf,              # accel, sensor axes
            "0", "0", "%.9g" % (roll_rate if phase in ("burn", "coast") else 0.0),  # gyro, rad/s; +z = nose
            str(mag_new), "20", "1", "45",       # mag, uT
            str(baro_new), "%.9g" % pressure_at(h), "20",
            "0", "0", "0", "0", "0",             # no encoder feedback
            cmds,
        ]
        yield ",".join(row)

        # Advance the vertical motion.
        if phase in ("burn", "coast"):
            a = sf - G
            v += a * dt
            h += v * dt
        elif phase == "descent":
            v = -descent_mps
            h += v * dt
        k += 1
        if landed_at is not None and t >= landed_at + 3.0:
            return


# --- Commands -------------------------------------------------------------------------

def looks_like_row(line):
    return bool(line) and line[0].isdigit()


def cmd_stream(args):
    if not args.synthetic and not args.log:
        raise SystemExit("give a LOG.csv or --synthetic")
    lines = (synthetic_lines(rate_hz=args.rate, roll_rate_dps=args.roll_rate) if args.synthetic
             else log_lines(args.log))
    out_path = args.output or ("stream_synthetic_replies.csv" if args.synthetic else
                               os.path.splitext(os.path.basename(args.log))[0] + "_stream.csv")

    ser = connect(args.port, "mode stream" + (" servos" if args.servos else ""), args.no_reset)
    buf = bytearray()
    # Everything before the ready line is the board's boot banners.
    end = time.monotonic() + READY_TIMEOUT_S
    while True:
        line = read_line(ser, max(0.0, end - time.monotonic()), buf)
        if line is None:
            raise SystemExit("the board never said '%s': did it miss the mode line? "
                             "(it waits only 1.5 s after setup)" % READY_LINE)
        print("board: %s" % line, file=sys.stderr)
        if line.strip() == READY_LINE:
            break

    sent = replies = errors = timeouts = 0
    first_error = None
    states = []
    state_col = None
    t_start = time.monotonic()
    with open(out_path, "w") as out:
        for line in lines:
            ser.write((line + "\n").encode())
            is_header = not looks_like_row(line)
            sent += 0 if is_header else 1
            reply = read_line(ser, args.timeout, buf)
            if reply is None:
                timeouts += 1
                print("no reply within %.1f s to: %s" % (args.timeout, line[:60]), file=sys.stderr)
                if timeouts >= 3:
                    print("giving up after 3 timeouts", file=sys.stderr)
                    break
                continue
            if reply.startswith("!ERR"):
                errors += 1
                first_error = first_error or "%s (to: %s)" % (reply, line[:60])
                continue
            out.write(reply + "\n")
            if is_header:
                cols = reply.split(",")
                state_col = cols.index("state") if "state" in cols else None
                continue
            replies += 1
            if state_col is not None:
                fields = reply.split(",")
                if state_col < len(fields) and (not states or states[-1][1] != fields[state_col]):
                    states.append((fields[0], fields[state_col]))
    ser.close()
    took = time.monotonic() - t_start

    names = ["Init", "PreFlight", "Boost", "Control", "Coast", "Chute", "Land"]
    print("sent %d rows, %d replies, %d errors, %d timeouts in %.1f s (%.0f rows/s)"
          % (sent, replies, errors, timeouts, took, sent / took if took > 0 else 0))
    if first_error:
        print("first error: %s" % first_error)
    if states:
        t0 = int(states[0][0])
        print("states: " + ", ".join("%s @ %.3f s" % (names[int(s)] if s.isdigit() and int(s) < len(names) else s,
                                                     (int(t) - t0) / 1e6) for t, s in states))
    print("replies saved to %s (python3 tools/flight_log.py summary %s)" % (out_path, out_path))
    print("OPEN LOOP: the input does not react to the board's commands.")
    return 0 if (errors == 0 and timeouts == 0) else 1


def cmd_replay(args):
    mode = "mode replay %d" % args.index + (" servos" if args.servos else "") + (" fast" if args.fast else "")
    ser = connect(args.port, mode, args.no_reset)
    buf = bytearray()
    print("board output (Ctrl-C to stop):", file=sys.stderr)
    try:
        while True:
            line = read_line(ser, 1.0, buf)
            if line is None:
                continue
            print(line)
            if line.startswith("# replay done") or "failed" in line:
                break
    except KeyboardInterrupt:
        pass
    ser.close()
    print("The replay's own log is the board's log file for that boot. Reboot normally and download it "
          "(pio run -e marsv21 -t dumplog), then: python3 tools/flight_log.py compare ORIGINAL.csv REPLAY.csv")
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--port", help="the board's serial port (default: find USB %04x:%04x)" % (STM_VID, STM_CDC_PID))
    p.add_argument("--no-reset", action="store_true", help="open the port now instead of waiting for a reset")
    sub = p.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("stream", help="stream a log (or a synthetic flight) in lockstep")
    s.add_argument("log", nargs="?", help="a flight log CSV (header + rows)")
    s.add_argument("--synthetic", action="store_true", help="a generated flight instead of a log")
    s.add_argument("--rate", type=float, default=200.0, help="synthetic sample rate, Hz (default 200)")
    s.add_argument("--servos", action="store_true", help="let the servos move")
    s.add_argument("--roll-rate", type=float, default=0.0, metavar="DPS",
                   help="synthetic: constant roll rate during burn and coast, deg/s "
                        "(+ = clockwise seen from the tail); for the control-sign check")
    s.add_argument("--timeout", type=float, default=2.0, help="seconds to wait for each reply")
    s.add_argument("-o", "--output", help="where to save the replies (a CSV log)")

    r = sub.add_parser("replay", help="start an SD replay of /logs/canardsLog<N>.csv")
    r.add_argument("index", type=int)
    r.add_argument("--servos", action="store_true", help="let the servos move")
    r.add_argument("--fast", action="store_true", help="as fast as the SD log keeps up")

    g = sub.add_parser("synthetic", help="print the synthetic flight's rows (no board needed)")
    g.add_argument("--rate", type=float, default=200.0)
    g.add_argument("--roll-rate", type=float, default=0.0, metavar="DPS",
                   help="synthetic: constant roll rate during burn and coast, deg/s "
                        "(+ = clockwise seen from the tail); for the control-sign check")

    args = p.parse_args(argv)
    if args.cmd == "synthetic":
        for line in synthetic_lines(rate_hz=args.rate, roll_rate_dps=args.roll_rate):
            print(line)
        return 0
    if args.cmd == "stream":
        return cmd_stream(args)
    return cmd_replay(args)


if __name__ == "__main__":
    sys.exit(main())
