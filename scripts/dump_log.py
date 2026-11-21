#!/usr/bin/env python3
"""Download CSV logs from a MARS board's SD card over USB.

Talks to src/logdump.cpp over the board's USB serial port. Works with the
normal flight and debug firmware; no special build is needed on the board.

  python scripts/dump_log.py              # previous boot's log
  python scripts/dump_log.py --index 7    # /logs/canardsLog7.csv
  python scripts/dump_log.py --list       # what's on the card

Also available as PlatformIO targets (see scripts/log_targets.py):
  pio run -t listlogs
  pio run -t dumplog                      # LOG_INDEX=7 pio run -t dumplog
"""

import argparse
import datetime
import os
import sys
import time

import serial
from serial.tools import list_ports

STM_VID = 0x0483
STM_CDC_PID = 0x5740
# Any rate but 1200: opening the port at 1200 baud reboots the board into
# its DFU bootloader (src/dfu_bootloader.cpp).
BAUD = 115200
REPLY_TIMEOUT_S = 10
DATA_TIMEOUT_S = 10  # longest gap allowed between chunks of a dump

DEFAULT_OUT_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "logs")


def find_port():
    for p in list_ports.comports():
        if p.vid == STM_VID and p.pid == STM_CDC_PID:
            return p.device
    return None


def read_marker(ser, prefixes):
    """Returns the first line starting with one of `prefixes`, skipping the
    debug output the firmware prints around it."""
    deadline = time.time() + REPLY_TIMEOUT_S
    while time.time() < deadline:
        line = ser.readline().decode("utf-8", "replace").strip()
        if line.startswith("---ERROR"):
            sys.exit("Board: " + line.strip("-")[len("ERROR "):])
        if line.startswith(prefixes):
            return line
    sys.exit("No reply from the board. Is it running firmware with logdump.cpp?")


def send(ser, cmd):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    ser.flush()


def list_logs(ser):
    send(ser, "list")
    read_marker(ser, ("---LIST BEGIN---",))
    entries = []
    while True:
        line = read_marker(ser, ("canardsLog", "---LIST END---"))
        if line == "---LIST END---":
            break
        parts = line.split()
        entries.append((parts[0], int(parts[1]), "live" in parts[2:]))

    def idx(e):
        digits = "".join(c for c in e[0] if c.isdigit())
        return int(digits) if digits else -1

    for name, size, live in sorted(entries, key=idx):
        print("%-20s %10d bytes%s" % (name, size, "  (this boot, live)" if live else ""))
    if not entries:
        print("No logs on the card.")


def dump_log(ser, index, out_dir):
    send(ser, "dump" if index is None else "dump %d" % index)
    header = read_marker(ser, ("---BEGIN ",))
    _, name, size = header.strip("-").split()
    size = int(size)

    data = bytearray()
    start = time.time()
    last_data = start
    while len(data) < size:
        chunk = ser.read(min(65536, size - len(data)))
        now = time.time()
        if chunk:
            data += chunk
            last_data = now
            pct = 100.0 * len(data) / size
            rate = len(data) / max(now - start, 1e-6) / 1024
            sys.stderr.write("\r%s: %d / %d bytes (%.0f%%, %.0f KiB/s)  "
                             % (name, len(data), size, pct, rate))
        elif now - last_data > DATA_TIMEOUT_S:
            sys.exit("\nTimed out after %d of %d bytes." % (len(data), size))
    sys.stderr.write("\n")

    trailer = read_marker(ser, ("---END---", "---ERROR"))

    os.makedirs(out_dir, exist_ok=True)
    # Log indexes get reused, so stamp the download time into the name.
    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    base, ext = os.path.splitext(name)
    path = os.path.join(out_dir, "%s_%s%s" % (base, stamp, ext))
    with open(path, "wb") as f:
        f.write(data)
    print("Saved %s (%d bytes, %.1f s)" % (path, size, time.time() - start))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="list the logs on the card")
    ap.add_argument("--index", type=int, help="log number to download "
                    "(default: the previous boot's log)")
    ap.add_argument("--port", help="serial port (default: find the MARS board)")
    ap.add_argument("--out", default=DEFAULT_OUT_DIR,
                    help="folder to save into (default: %(default)s)")
    args = ap.parse_args()

    port = args.port or find_port()
    if not port:
        sys.exit("No MARS board found on USB. Plug it in, or pass --port.")

    with serial.Serial(port, BAUD, timeout=0.5) as ser:
        if args.list:
            list_logs(ser)
        else:
            dump_log(ser, args.index, args.out)


if __name__ == "__main__":
    main()
