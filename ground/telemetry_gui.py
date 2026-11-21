#!/usr/bin/env python3
"""
Canards ground station: receive radio telemetry, log it, plot it, send commands.

QUICK START
    pip install -r ground/requirements.txt
    python3 ground/telemetry_gui.py              # pick a serial port, Connect
    python3 ground/telemetry_gui.py --list       # just list serial ports
    python3 ground/telemetry_gui.py --usb --flight   # bench: the board's USB port, no radio,
                                                     # plots for the flight program (env marsv21)

    Choose "SIMULATED" in the port list to try the GUI without hardware: it
    produces fake flight data in exactly the format the rocket sends.

WHAT YOU NEED
    A serial port that carries the raw radio byte stream: e.g. an E22 module
    (same config as include/RadioConfigs.h, M0/M1 low = normal mode) on a
    USB-UART adapter, at SERIAL_BAUD below.

WHAT IT DOES
    * Saves everything to LOG_DIR/<date_time>/:
        telemetry.csv   one row per data line, with the PC's receive time
                        (telemetry_2.csv, ... if the rocket's columns change)
        raw.bin         every byte received, untouched, for re-parsing later
        events.log      commands sent, replies, link errors
    * Shows the latest value of every column (and of the DERIVED values).
    * Plots the series listed in PLOTS.
    * One button per entry in COMMANDS.
    * With --usb: the SD replay panel, the ONLY way to run an SD replay. It
      copies a flight CSV onto the SD card while the card is in this computer,
      lists the logs on the board's card, and starts a replay: connected, with
      the board in Init, it sends "replay <N> [servos]"; the board restarts
      itself into the replay (src/main.cpp) and the GUI reconnects and shows
      the replay's telemetry. No unplugging.

HOW TO CHANGE IT  (everything is in the CONFIGURATION section below)
    * Plot something else:  edit PLOTS. A series is any column name the rocket
                            sends (see the header / values table) or any key
                            of DERIVED.
    * Plot a computed value: add a function to DERIVED, then name it in PLOTS.
    * Add a command button: add it to COMMANDS, and to the firmware with
                            telemetry.addCommand() in src/main.cpp.
    * Columns themselves come from the firmware (tlmColumns in src/main.cpp);
      nothing here needs changing when columns are added or removed.

PROTOCOL  (mirrors src/telemetry.h)
    Every message is one ASCII line:  $<type>,<field>,...*<checksum>\\n
    checksum = 2 hex digits, XOR of all bytes between '$' and '*' (NMEA style).
        $H,name0,name1,...      header (column names), re-sent every ~5 s
        $D,value0,value1,...    data row
        $A,command,result       command reply: OK / REJECTED / UNKNOWN
        $C,command              ground -> rocket command
    On air, the radio HAL wraps each message chunk (max 232 bytes) as
    <callsign><1 length byte><payload>; see Deframer.
"""

import argparse
import math
import os
import queue
import shutil
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import tkinter as tk
from tkinter import ttk, messagebox, filedialog

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure

import serial
import serial.tools.list_ports


# =============================================================================
# CONFIGURATION: edit this section to change what is shown and graphed
# =============================================================================

SERIAL_BAUD = 115200          # must match SERIALSPEED in include/RadioConfigs.h

# Radio HAL framing. Each chunk on air is <CALLSIGN><length byte><payload>.
# Set RADIO_FRAMING = False if your ground radio/relay already strips this from
# what it receives and hands over plain "$...\n" lines. Commands we send are
# always framed: the rocket's radio driver only accepts framed messages.
CALLSIGN = b"KV0R"            # callsign the rocket transmits with
GROUND_CALLSIGN = b"KV0R"     # callsign we transmit commands with (yours!)
RADIO_FRAMING = True
MAX_MESSAGE_SIZE = 232        # largest payload per chunk (LoRaE22.h)

# --usb: talk to the board's own USB port instead of a ground radio (the flight
# program's USB bench console, src/Platform/UsbConsole.h). The same lines, with
# no radio framing in either direction. Set from the command line.
USB_DIRECT = False

LOG_DIR = Path(__file__).resolve().parent / "ground_logs"

HISTORY_SECONDS = 60          # how much data the plots keep / show
PLOT_REFRESH_MS = 250         # how often the plots redraw
STALE_AFTER_S = 2.0           # "last packet" turns red after this long

SEA_LEVEL_HPA = 1013.25       # reference pressure for baro_alt_m

# Values computed on the ground from each received row. Each function gets the
# row as a dict {column name: number} and returns a number. They show up in the
# values table (marked "derived") and can be plotted like any column. A
# function that fails (e.g. a column it needs isn't sent) just shows blank.
DERIVED = {
    # x axis for every plot: rocket time in seconds
    "time_s":         lambda r: r["time_ms"] / 1000.0,
    # total acceleration, g (1.0 at rest)
    "accel_g":        lambda r: math.sqrt(r["ax_mg"] ** 2 + r["ay_mg"] ** 2 + r["az_mg"] ** 2) / 1000.0,
    "az_g":           lambda r: r["az_mg"] / 1000.0,
    # body rates, deg/s
    "gx_dps":         lambda r: r["gx_mdps"] / 1000.0,
    "gy_dps":         lambda r: r["gy_mdps"] / 1000.0,
    "gz_dps":         lambda r: r["gz_mdps"] / 1000.0,
    # barometric altitude, standard atmosphere, m
    "baro_alt_m":     lambda r: 44330.0 * (1.0 - (r["pressure_hpa"] / SEA_LEVEL_HPA) ** 0.1903),
    # GPS vertical speed, up positive, m/s
    "gps_vel_up_mps": lambda r: -r["gps_vel_d_mmps"] / 1000.0,
}

# Column (or derived value) used as the x axis of every plot.
X_AXIS = "time_s"

# One entry per plot, top to bottom. "series" lists column names or DERIVED
# keys; each gets a line, colored in list order. Keep each plot to one unit
# (one y axis), and at most ~4 series so the lines stay distinguishable.
PLOTS = [
    {"title": "Altitude",          "ylabel": "m",   "series": ["baro_alt_m", "gps_alt_m"]},
    {"title": "Vertical velocity", "ylabel": "m/s", "series": ["gps_vel_up_mps"]},
    {"title": "Acceleration",      "ylabel": "g",   "series": ["accel_g", "az_g"]},
    {"title": "Angular rate",      "ylabel": "deg/s", "series": ["gx_dps", "gy_dps", "gz_dps"]},
]

# The flight program (src/main.cpp, env marsv21) sends its own columns, not the
# bring-up sketch's: `--flight` plots these instead of PLOTS.
FLIGHT_PLOTS = [
    {"title": "Altitude (estimate)", "ylabel": "m",   "series": ["alt_m"]},
    {"title": "Speed (estimate)",    "ylabel": "m/s", "series": ["speed"]},
    {"title": "Tilt",                "ylabel": "deg", "series": ["tilt_deg"]},
    {"title": "Accel, body axes",    "ylabel": "g",   "series": ["sf_g", "fx_g", "fy_g", "fz_g"]},
    {"title": "Rate, body axes",     "ylabel": "deg/s", "series": ["p_dps", "q_dps", "r_dps"]},
]

# Line colors, used in order within each plot (a fixed order, so the first
# series of every plot is always blue, the second orange, ...).
SERIES_COLORS = ["#2a78d6", "#eb6834", "#1baf7a", "#eda100"]

# Command buttons. "name" is sent as "$C,<name>"; it must match a command the
# firmware registers. "confirm" (optional) asks before sending.
COMMANDS = [
    {"name": "armFlight",    "label": "ARM FLIGHT",
     "confirm": "Arm the rocket for flight?\n\nThis cannot be undone without a reboot."},
    {"name": "actuatorTest", "label": "Actuator test",
     "confirm": "Run the actuator test? Keep clear of the canards."},
]

SIMULATED_PORT = "SIMULATED"

# SD replay (--usb only). "replay <N> [servos]" over the USB console makes the
# board restart itself into the replay (src/main.cpp, requestReplayReboot).
BOARD_VID, BOARD_PID = 0x0483, 0x5740     # the board's USB CDC port (STM32)
REPLAY_RESTART_S = 5          # the board drops off USB this soon after the command
REPLAY_WAIT_S = 30            # and comes back within this
SD_LOG_FOLDER = "logs"        # LOG_FOLDER_NAME in the firmware
SD_MAX_LOG_IDX = 100          # MAX_LOG_FILE_IDX in src/sdcard.cpp (wraps after it)


# =============================================================================
# PROTOCOL: building and parsing lines (matches src/telemetry.cpp)
# =============================================================================

def checksum(body: str) -> int:
    """XOR of every character of `body` (the text between '$' and '*')."""
    cs = 0
    for ch in body.encode("ascii"):
        cs ^= ch
    return cs


def make_line(msg_type: str, fields) -> bytes:
    """Builds a complete line, e.g. make_line("C", ["armFlight"])."""
    body = ",".join([msg_type] + [str(f) for f in fields])
    return f"${body}*{checksum(body):02X}\n".encode("ascii")


def frame(payload: bytes, callsign: bytes = GROUND_CALLSIGN) -> bytes:
    """Wraps a payload the way the radio HAL does: callsign + length + data,
    split into chunks of at most MAX_MESSAGE_SIZE bytes."""
    out = b""
    for i in range(0, len(payload), MAX_MESSAGE_SIZE):
        chunk = payload[i:i + MAX_MESSAGE_SIZE]
        out += callsign + bytes([len(chunk)]) + chunk
    return out


class BadLine(ValueError):
    """A line that failed its checksum or is malformed."""


def parse_line(line: str):
    """'$D,1,2*XX' -> ('D', ['1', '2']). Raises BadLine if it doesn't check out."""
    if not line.startswith("$") or "*" not in line:
        raise BadLine(f"malformed: {line!r}")
    body, _, cs_text = line[1:].rpartition("*")
    try:
        cs = int(cs_text, 16)
    except ValueError:
        raise BadLine(f"bad checksum field: {line!r}")
    if cs != checksum(body):
        raise BadLine(f"checksum mismatch: {line!r}")
    msg_type, *fields = body.split(",")
    return msg_type, fields


def to_number(text: str):
    """'12' -> 12, '1.5' -> 1.5, anything else stays text."""
    try:
        return int(text)
    except ValueError:
        try:
            return float(text)
        except ValueError:
            return text


class Deframer:
    """Strips the radio HAL's <callsign><length> framing from a byte stream.

    Feed it bytes as they arrive; it returns the payload bytes found so far,
    joined together. A lost or damaged chunk just means the line it belonged
    to fails its checksum later.
    """

    def __init__(self, callsign: bytes):
        self.callsign = callsign
        self.buf = bytearray()

    def feed(self, data: bytes) -> bytes:
        if not RADIO_FRAMING or USB_DIRECT:
            return data
        self.buf += data
        out = bytearray()
        cs = self.callsign
        while True:
            start = self.buf.find(cs)
            if start < 0:
                # Keep a possible partial callsign at the end for next time.
                del self.buf[:max(0, len(self.buf) - (len(cs) - 1))]
                break
            header_end = start + len(cs) + 1
            if len(self.buf) < header_end:
                del self.buf[:start]
                break
            length = self.buf[header_end - 1]
            payload = self.buf[header_end:header_end + length]
            # A callsign inside the payload means this chunk was cut short
            # (lost packet): keep what came before it, resync at the callsign.
            cut = payload.find(cs)
            if cut >= 0:
                out += payload[:cut]
                del self.buf[:header_end + cut]
                continue
            if len(payload) < length:
                del self.buf[:start]
                break                       # wait for the rest of the chunk
            out += payload
            del self.buf[:header_end + length]
        return bytes(out)


class LineSplitter:
    """Turns a stream of payload bytes into complete '$...' lines."""

    MAX_BUFFER = 8192

    def __init__(self):
        self.buf = ""

    def feed(self, data: bytes):
        self.buf += data.decode("ascii", errors="replace")
        *lines, self.buf = self.buf.split("\n")
        if len(self.buf) > self.MAX_BUFFER:  # garbage with no newline
            self.buf = ""
        for line in lines:
            start = line.rfind("$")          # the last '$' starts the real line
            if start >= 0:
                yield line[start:].rstrip("\r")
            elif USB_DIRECT and line.strip():
                yield line.rstrip("\r")      # board text (banners, log list): no radio noise on USB


# =============================================================================
# LINKS: where bytes come from (a serial port, or the simulator)
# =============================================================================

class SerialLink:
    def __init__(self, port: str, baud: int = SERIAL_BAUD, ser=None):
        # `ser`: a port that is already open (the SD replay opens it itself).
        self.ser = ser if ser is not None else serial.Serial(port, baud, timeout=0.1)

    def read(self) -> bytes:
        return self.ser.read(self.ser.in_waiting or 1)

    def write(self, data: bytes):
        self.ser.write(data)

    def close(self):
        self.ser.close()


class SimulatedLink:
    """Pretends to be the rocket: sends header/data/replies in the real format,
    framed and chunked like the radio HAL, and answers commands."""

    COLUMNS = ["time_ms", "seq", "armed", "act_test", "ax_mg", "ay_mg", "az_mg",
               "gx_mdps", "gy_mdps", "gz_mdps", "pressure_hpa", "temp_c",
               "gps_fix", "gps_sats", "gps_lat_deg", "gps_lon_deg", "gps_alt_m",
               "gps_vel_d_mmps", "sensor_ok"]

    def __init__(self):
        self.t0 = time.monotonic()
        self.seq = 0
        self.armed = 0
        self.act_test_until = 0.0
        self.next_row = 0.0
        self.next_header = 0.5        # first header "missed": arrives after rows
        self.out = queue.Queue()
        self.rx = LineSplitter()
        self.deframe = Deframer(GROUND_CALLSIGN)

    def _send(self, line: bytes):
        self.out.put(frame(line, CALLSIGN) if RADIO_FRAMING else line)

    def read(self) -> bytes:
        time.sleep(0.02)
        t = time.monotonic() - self.t0
        if t >= self.next_header:
            self.next_header += 5.0
            self._send(make_line("H", self.COLUMNS))
        if t >= self.next_row:
            self.next_row += 0.25
            self._send(make_line("D", self._row(t)))
        chunks = []
        while not self.out.empty():
            chunks.append(self.out.get())
        return b"".join(chunks)

    def _row(self, t):
        # A crude flight: 10 s on the pad, 3 s burn at ~8 g, coast to apogee
        # (~3300 m AGL at 37.5 s), then descent at 20 m/s.
        tf = max(0.0, t - 10.0)
        tb = tf - 3.0
        if tf < 3.0:
            alt, vel, acc = 40 * tf ** 2, 80 * tf, 9.0
        elif tb < 24.5:
            alt, vel, acc = 360 + 240 * tb - 4.9 * tb ** 2, 240 - 9.8 * tb, 0.0
        else:
            alt, vel, acc = 3299 - 20 * (tb - 24.5), -20.0, 1.0
        if alt <= 0 and tf > 0:
            alt, vel, acc = 0.0, 0.0, 1.0
        alt = max(alt, 0.0)
        pressure = SEA_LEVEL_HPA * (1 - alt / 44330.0) ** (1 / 0.1903)
        az = (acc if tf > 0 else 1.0) * 1000 + math.sin(t * 7) * 30
        self.seq += 1
        return [int(t * 1000), self.seq, self.armed, int(t < self.act_test_until),
                int(math.sin(t) * 40), int(math.cos(t) * 40), int(az),
                int(math.sin(t * 2) * 5000), int(math.cos(t * 3) * 3000), int(tf * 20000 % 90000),
                f"{pressure:.2f}", "21.5", 3, 9, "42.280638", "-71.806624",
                f"{alt + 160:.1f}", int(-vel * 1000), 15]

    def write(self, data: bytes):
        for line in self.rx.feed(self.deframe.feed(data)):
            try:
                msg_type, fields = parse_line(line)
            except BadLine:
                continue
            if msg_type != "C" or not fields:
                continue
            name = fields[0]
            if name == "armFlight":
                self.armed, result = 1, "OK"
            elif name == "actuatorTest":
                ok = not self.armed and time.monotonic() - self.t0 > self.act_test_until
                if ok:
                    self.act_test_until = time.monotonic() - self.t0 + 3.0
                result = "OK" if ok else "REJECTED"
            else:
                name, result = "?", "UNKNOWN"
            self._send(make_line("A", [name, result]))

    def close(self):
        pass


def open_link(port: str):
    return SimulatedLink() if port == SIMULATED_PORT else SerialLink(port)


def list_ports():
    return [p.device for p in serial.tools.list_ports.comports()]


# =============================================================================
# SD REPLAY: getting a flight onto the card, and starting the board in replay
# =============================================================================

def find_board_port():
    for p in serial.tools.list_ports.comports():
        if p.vid == BOARD_VID and p.pid == BOARD_PID:
            return p.device
    return None


def find_sd_cards():
    """Mounted volumes that look like the board's SD card (a logs/ folder)."""
    volumes = Path("/Volumes")
    if not volumes.is_dir():
        return []
    return [v for v in sorted(volumes.iterdir()) if (v / SD_LOG_FOLDER).is_dir()]


def free_log_index(card: Path):
    """The highest log number with no file on the card, or None. High numbers
    are the last the board's own logs reach before they wrap."""
    used = {p.stem for p in (card / SD_LOG_FOLDER).iterdir()}
    for idx in range(SD_MAX_LOG_IDX - 1, -1, -1):
        if f"canardsLog{idx}" not in used:
            return idx
    return None


def wait_for_restart(status, stop):
    """Background thread, after the "replay" command went out: waits for the
    board's USB port to go away (the board restarts itself into the replay)
    and come back, then opens it. Returns the open port. `status(text)`
    reports progress; `stop()` cancels."""
    def wait(cond, timeout_s, what):
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            if stop():
                raise RuntimeError("cancelled")
            v = cond()
            if v:
                return v
            time.sleep(0.02)
        raise RuntimeError(f"timed out waiting for {what}")

    status("Board restarting into the replay...")
    wait(lambda: not find_board_port(), REPLAY_RESTART_S,
         "the board to restart (see the events for an ERROR line)")
    port = wait(find_board_port, REPLAY_WAIT_S, "the board's USB port to come back")
    end = time.monotonic() + 2.0
    while True:
        try:
            return serial.Serial(port, SERIAL_BAUD, timeout=0.1)
        except (OSError, serial.SerialException):
            if time.monotonic() > end:
                raise
            time.sleep(0.02)


# =============================================================================
# LOGGING: everything received goes to disk
# =============================================================================

class SessionLog:
    """One folder per connection: telemetry CSV(s), raw bytes, event log."""

    def __init__(self):
        self.dir = LOG_DIR / datetime.now().strftime("%Y%m%d_%H%M%S")
        self.dir.mkdir(parents=True, exist_ok=True)
        self.raw = open(self.dir / "raw.bin", "wb")
        self.events = open(self.dir / "events.log", "a", buffering=1)
        self.csv = None
        self.csv_count = 0

    def new_csv(self, columns):
        """Starts a CSV for a (new) set of columns."""
        if self.csv:
            self.csv.close()
        self.csv_count += 1
        name = "telemetry.csv" if self.csv_count == 1 else f"telemetry_{self.csv_count}.csv"
        self.csv = open(self.dir / name, "w", buffering=1)
        self.csv.write(",".join(["pc_time"] + list(columns)) + "\n")
        self.event(f"logging columns to {name}")

    def row(self, pc_time: float, fields):
        if self.csv:
            stamp = datetime.fromtimestamp(pc_time).isoformat(timespec="milliseconds")
            self.csv.write(",".join([stamp] + list(fields)) + "\n")

    def raw_bytes(self, data: bytes):
        self.raw.write(data)

    def event(self, text: str):
        self.events.write(f"{datetime.now().isoformat(timespec='milliseconds')}  {text}\n")

    def close(self):
        for f in (self.raw, self.events, self.csv):
            if f:
                f.close()


# =============================================================================
# GUI
# =============================================================================

class TelemetryApp:
    MAX_ROWS_BEFORE_HEADER = 500  # rows kept while waiting for the first header

    def __init__(self, root: tk.Tk):
        self.root = root
        root.title("Canards ground station")

        # Link state
        self.link = None
        self.reader = None
        self.rx_queue = queue.Queue()   # bytes from the reader thread
        self.log = None
        self.deframer = Deframer(CALLSIGN)
        self.splitter = LineSplitter()

        # Telemetry state
        self.columns = None             # names from the last $H
        self.waiting_rows = []          # (pc_time, fields) received before any $H
        self.latest = {}                # name -> latest value (columns + derived)
        self.history = {}               # name -> deque of values, for plots
        self.stats = {"rows": 0, "headers": 0, "bad": 0, "lost": 0}
        self.last_seq = None
        self.last_rx = None
        self.warned_missing = set()

        # SD replay state (--usb only)
        self.replay_queue = queue.Queue()  # ("status"|"ready"|"failed", payload) from its thread
        self.replay_thread = None
        self.replay_cancel = False
        self.in_listing = False         # between ---LIST BEGIN--- and ---LIST END---

        self._build_ui()
        self.root.after(50, self._poll)
        self.root.after(PLOT_REFRESH_MS, self._redraw_plots)
        root.protocol("WM_DELETE_WINDOW", self._on_close)

    # ---------------------------------------------------------------- layout
    def _build_ui(self):
        top = ttk.Frame(self.root, padding=4)
        top.pack(side=tk.TOP, fill=tk.X)
        ttk.Label(top, text="Port:").pack(side=tk.LEFT)
        self.port_var = tk.StringVar()
        self.port_box = ttk.Combobox(top, textvariable=self.port_var, width=24)
        self.port_box.pack(side=tk.LEFT, padx=4)
        ttk.Button(top, text="Refresh", command=self._refresh_ports).pack(side=tk.LEFT)
        self.connect_btn = ttk.Button(top, text="Connect", command=self._toggle_connect)
        self.connect_btn.pack(side=tk.LEFT, padx=4)
        self.status_var = tk.StringVar(value="Disconnected")
        ttk.Label(top, textvariable=self.status_var).pack(side=tk.LEFT, padx=8)
        self._refresh_ports()

        body = ttk.PanedWindow(self.root, orient=tk.HORIZONTAL)
        body.pack(fill=tk.BOTH, expand=True)

        # Left column: commands, link stats, values, event log
        left = ttk.Frame(body, padding=4)
        body.add(left, weight=1)

        cmd_frame = ttk.LabelFrame(left, text="Commands", padding=4)
        cmd_frame.pack(fill=tk.X)
        self.cmd_buttons = []
        for cmd in COMMANDS:
            b = ttk.Button(cmd_frame, text=cmd["label"], state=tk.DISABLED,
                           command=lambda c=cmd: self._send_command(c))
            b.pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
            self.cmd_buttons.append(b)

        if USB_DIRECT:
            self._build_replay_panel(left)

        link_frame = ttk.LabelFrame(left, text="Link", padding=4)
        link_frame.pack(fill=tk.X, pady=4)
        self.link_var = tk.StringVar()
        self.age_label = tk.Label(link_frame, text="last packet: -", anchor="w")
        self.age_label.pack(fill=tk.X)
        ttk.Label(link_frame, textvariable=self.link_var, anchor="w").pack(fill=tk.X)

        val_frame = ttk.LabelFrame(left, text="Values", padding=4)
        val_frame.pack(fill=tk.BOTH, expand=True)
        self.values = ttk.Treeview(val_frame, columns=("value",), height=20)
        self.values.heading("#0", text="Name")
        self.values.heading("value", text="Value")
        self.values.column("#0", width=150)
        self.values.column("value", width=120, anchor="e")
        self.values.pack(fill=tk.BOTH, expand=True)
        self.values_group = self.values.insert("", tk.END, text="telemetry", open=True)
        self.derived_group = self.values.insert("", tk.END, text="derived", open=True)
        self.value_rows = {}            # name -> tree item

        ev_frame = ttk.LabelFrame(left, text="Events", padding=4)
        ev_frame.pack(fill=tk.BOTH, pady=4)
        self.events = tk.Text(ev_frame, height=8, width=50, state=tk.DISABLED)
        self.events.pack(fill=tk.BOTH, expand=True)

        # Right column: plots
        right = ttk.Frame(body)
        body.add(right, weight=3)
        self.fig = Figure(figsize=(8, 2.2 * len(PLOTS)), layout="constrained")
        self.axes = []
        self.lines = []                 # (line, series name) per plotted series
        for i, spec in enumerate(PLOTS):
            ax = self.fig.add_subplot(len(PLOTS), 1, i + 1,
                                      sharex=self.axes[0] if self.axes else None)
            ax.set_title(spec["title"], loc="left", fontsize=10)
            ax.set_ylabel(spec.get("ylabel", ""))
            ax.grid(True, color="#e6e5e0", linewidth=0.8)
            for side in ("top", "right"):
                ax.spines[side].set_visible(False)
            for j, name in enumerate(spec["series"]):
                (line,) = ax.plot([], [], linewidth=2, label=name,
                                  color=SERIES_COLORS[j % len(SERIES_COLORS)])
                self.lines.append((line, name))
            if len(spec["series"]) > 1:
                ax.legend(loc="upper left", fontsize=8, frameon=False)
            self.axes.append(ax)
        if self.axes:
            self.axes[-1].set_xlabel(X_AXIS)
        self.canvas = FigureCanvasTkAgg(self.fig, master=right)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _build_replay_panel(self, parent):
        frame = ttk.LabelFrame(parent, text="SD replay", padding=4)
        frame.pack(fill=tk.X, pady=4)
        row1 = ttk.Frame(frame)
        row1.pack(fill=tk.X)
        ttk.Button(row1, text="Copy flight to SD card...",
                   command=self._copy_flight_to_card).pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
        self.list_btn = ttk.Button(row1, text="List SD logs", state=tk.DISABLED,
                                   command=self._list_logs)
        self.list_btn.pack(side=tk.LEFT, padx=2, expand=True, fill=tk.X)
        row2 = ttk.Frame(frame)
        row2.pack(fill=tk.X, pady=(4, 0))
        ttk.Label(row2, text="Log #").pack(side=tk.LEFT)
        self.replay_idx = tk.StringVar()
        ttk.Entry(row2, textvariable=self.replay_idx, width=5).pack(side=tk.LEFT, padx=4)
        self.replay_servos = tk.BooleanVar(value=False)
        ttk.Checkbutton(row2, text="Move servos", variable=self.replay_servos).pack(side=tk.LEFT)
        self.replay_btn = ttk.Button(row2, text="Start replay", command=self._start_replay)
        self.replay_btn.pack(side=tk.LEFT, padx=4, expand=True, fill=tk.X)
        self.replay_status = tk.StringVar(value="")
        ttk.Label(frame, textvariable=self.replay_status, wraplength=380,
                  foreground="#b45f06").pack(fill=tk.X, pady=(4, 0))

    def _copy_flight_to_card(self):
        cards = find_sd_cards()
        if not cards:
            messagebox.showerror("No SD card", "Put the board's SD card in this computer first "
                                 f"(a volume with a {SD_LOG_FOLDER}/ folder).")
            return
        card = cards[0]
        src = filedialog.askopenfilename(title="Flight CSV to replay",
                                         filetypes=[("Flight logs", "*.csv"), ("All files", "*")])
        if not src:
            return
        idx = free_log_index(card)
        if idx is None:
            messagebox.showerror("SD card full", f"Every log number on {card} is in use.")
            return
        dest = card / SD_LOG_FOLDER / f"canardsLog{idx}.csv"
        if dest.exists():               # free_log_index said no; never overwrite
            messagebox.showerror("Not copied", f"{dest} already exists.")
            return
        try:
            shutil.copyfile(src, dest)
            if Path(src).read_bytes() != dest.read_bytes():
                raise OSError("the copy on the card does not match the original")
            os.sync()
        except OSError as e:
            messagebox.showerror("Copy failed", str(e))
            return
        self.replay_idx.set(str(idx))
        self.event(f"copied {Path(src).name} to {dest}")
        messagebox.showinfo("Copied", f"Copied to {dest} (log #{idx}).\n\n"
                            "Eject the card and put it back in the board. Power it up, Connect, "
                            "then press Start replay.\n\n"
                            "The board's own logs overwrite this number once they count up to it.")

    def _list_logs(self):
        if not self.link:
            return
        try:
            self.link.write(b"list\n")
            self.event("asked the board for its SD logs")
        except Exception as e:
            self.event(f"list failed: {e}")

    def _start_replay(self):
        if self.replay_thread and self.replay_thread.is_alive():
            self.replay_cancel = True   # the button reads "Cancel" while waiting
            return
        try:
            idx = int(self.replay_idx.get())
            if not 0 <= idx <= SD_MAX_LOG_IDX:
                raise ValueError
        except ValueError:
            messagebox.showerror("Log number", f"Type the log number to replay (0-{SD_MAX_LOG_IDX}).")
            return
        if not self.link:
            messagebox.showerror("Not connected", "Connect to the board first (it must be in Init, not armed).")
            return
        servos = self.replay_servos.get()
        if servos and not messagebox.askyesno(
                "Move servos", "The canards will move during the replay.\n\nKeep clear of the canards."):
            return
        command = f"replay {idx}" + (" servos" if servos else "")
        try:
            self.link.write((command + "\n").encode())
        except Exception as e:
            self.event(f"replay: send failed: {e}")
            return
        self.event(f"sent '{command}'")
        self.replay_cancel = False
        self.replay_btn.configure(text="Cancel")
        # Keep reading until the board drops off USB, so an ERROR reply still
        # shows; the reader thread ends on its own when the port goes away.

        def run():
            put = self.replay_queue.put
            try:
                ser = wait_for_restart(lambda s: put(("status", s)), lambda: self.replay_cancel)
                put(("ready", (ser, command)))
            except Exception as e:
                put(("failed", str(e)))

        self.replay_thread = threading.Thread(target=run, daemon=True)
        self.replay_thread.start()

    def _poll_replay(self):
        try:
            while True:
                kind, payload = self.replay_queue.get_nowait()
                if kind == "status":
                    self.replay_status.set(payload)
                elif kind == "failed":
                    self.replay_btn.configure(text="Start replay")
                    self.replay_status.set(f"Replay not started: {payload}")
                    self.event(f"replay not started: {payload}")
                elif kind == "ready":
                    ser, command = payload
                    if self.link:
                        self._disconnect()  # the old session, if the reader has not ended it yet
                    self.replay_btn.configure(text="Start replay")
                    self.replay_status.set("Board restarted. Watch the events for 'mode: replay'.")
                    self._attach_link(SerialLink(ser.port, ser=ser), ser.port)
                    self.event(f"reconnected after '{command}'")
        except queue.Empty:
            pass

    def _handle_board_text(self, line: str):
        """USB only: a line that is not a $-message (boot banners, replay
        results, the log listing). Shown in the events."""
        text = line.strip()
        if not text:
            return
        if text == "---LIST BEGIN---":
            self.in_listing = True
            self.event("SD logs on the board:")
            return
        if text == "---LIST END---":
            self.in_listing = False
            return
        self.event(("    " if self.in_listing else "board: ") + text[:200])
        if USB_DIRECT and text.startswith(("mode: replay", "# replay done")):
            self.replay_status.set(text[:200])

    def _refresh_ports(self):
        ports = list_ports() + [SIMULATED_PORT]
        self.port_box["values"] = ports
        if not self.port_var.get():
            self.port_var.set(ports[0])

    def event(self, text: str):
        """Adds a line to the on-screen event list and to events.log."""
        stamp = datetime.now().strftime("%H:%M:%S")
        self.events.configure(state=tk.NORMAL)
        self.events.insert(tk.END, f"{stamp}  {text}\n")
        self.events.see(tk.END)
        self.events.configure(state=tk.DISABLED)
        if self.log:
            self.log.event(text)

    # -------------------------------------------------------------- the link
    def _toggle_connect(self):
        if self.link:
            self._disconnect()
            return
        port = self.port_var.get()
        try:
            link = open_link(port)
        except Exception as e:  # serial errors come in several types
            messagebox.showerror("Connect failed", str(e))
            return
        self._attach_link(link, port)

    def _attach_link(self, link, port: str):
        """Starts a session on an open link: the Connect button, or the SD
        replay once it has sent its mode line."""
        self.link = link
        self.in_listing = False
        self.log = SessionLog()
        self.deframer = Deframer(CALLSIGN)
        self.splitter = LineSplitter()
        self.columns = None
        self.last_seq = None
        self.reader = threading.Thread(target=self._reader, args=(self.link,), daemon=True)
        self.reader.start()
        self.connect_btn.configure(text="Disconnect")
        for b in self.cmd_buttons:
            b.configure(state=tk.NORMAL)
        if USB_DIRECT:
            self.list_btn.configure(state=tk.NORMAL)
        self.status_var.set(f"Connected to {port}, logging to {self.log.dir}")
        self.event(f"connected to {port}")

    def _disconnect(self):
        link, self.link = self.link, None   # tells the reader thread to stop
        if self.reader:
            self.reader.join(timeout=1)
        try:
            link.close()
        except Exception:                # the port may already be gone (board restarted)
            pass
        self.event("disconnected")
        self.log.close()
        self.log = None
        self.connect_btn.configure(text="Connect")
        for b in self.cmd_buttons:
            b.configure(state=tk.DISABLED)
        if USB_DIRECT:
            self.list_btn.configure(state=tk.DISABLED)
        self.status_var.set("Disconnected")

    def _reader(self, link):
        """Background thread: only moves bytes from the link to rx_queue."""
        while self.link is link:
            try:
                data = link.read()
            except Exception as e:
                self.rx_queue.put(e)
                return
            if data:
                self.rx_queue.put(data)

    def _send_command(self, cmd):
        if "confirm" in cmd and not messagebox.askyesno("Confirm", cmd["confirm"]):
            return
        try:
            line = make_line("C", [cmd["name"]])
            self.link.write(line if USB_DIRECT else frame(line))
            self.event(f"sent {cmd['name']}")
        except Exception as e:
            self.event(f"send {cmd['name']} failed: {e}")

    # ----------------------------------------------------- incoming messages
    def _poll(self):
        """Runs every 50 ms on the GUI thread: handles everything received."""
        try:
            while True:
                item = self.rx_queue.get_nowait()
                if isinstance(item, Exception):
                    self.event(f"link error: {item}")
                    if self.link:
                        self._disconnect()
                    continue
                if self.log:
                    self.log.raw_bytes(item)
                for line in self.splitter.feed(self.deframer.feed(item)):
                    self._handle_line(line)
        except queue.Empty:
            pass
        if USB_DIRECT:
            self._poll_replay()
        self._update_link_status()
        self.root.after(50, self._poll)

    def _handle_line(self, line: str):
        if USB_DIRECT and not line.lstrip().startswith("$"):
            self._handle_board_text(line)
            return
        try:
            msg_type, fields = parse_line(line)
        except BadLine:
            self.stats["bad"] += 1
            return
        now = time.time()
        self.last_rx = now
        if msg_type == "H":
            self._handle_header(fields)
        elif msg_type == "D":
            self._handle_row(now, fields)
        elif msg_type == "A" and len(fields) >= 2:
            self.event(f"reply: {fields[0]} -> {fields[1]}")

    def _handle_header(self, names):
        self.stats["headers"] += 1
        if names == self.columns:
            return                      # the periodic repeat; nothing new
        if self.columns is not None:
            self.event("rocket's columns changed, starting a new CSV")
        self.columns = names
        self.history.clear()
        self.last_seq = None
        if self.log:
            self.log.new_csv(names)
        self.event(f"header: {len(names)} columns")
        # Rows that arrived before we knew the column names.
        waiting, self.waiting_rows = self.waiting_rows, []
        for pc_time, fields in waiting:
            self._handle_row(pc_time, fields)

    def _handle_row(self, pc_time, fields):
        if self.columns is None:
            if len(self.waiting_rows) < self.MAX_ROWS_BEFORE_HEADER:
                self.waiting_rows.append((pc_time, fields))
            return
        if len(fields) != len(self.columns):
            self.stats["bad"] += 1
            return
        self.stats["rows"] += 1
        if self.log:
            self.log.row(pc_time, fields)

        row = {name: to_number(v) for name, v in zip(self.columns, fields)}
        for name, fn in DERIVED.items():
            try:
                row[name] = fn(row)
            except (KeyError, TypeError, ValueError, ZeroDivisionError):
                row[name] = None

        seq = row.get("seq")
        if isinstance(seq, int):
            if self.last_seq is not None and seq > self.last_seq + 1:
                self.stats["lost"] += seq - self.last_seq - 1
            self.last_seq = seq

        self.latest = row
        for name, value in row.items():
            self.history.setdefault(name, deque()).append(value)
        self._trim_history()
        self._update_values_table()

    def _trim_history(self):
        """Drops points older than HISTORY_SECONDS (by the X_AXIS value)."""
        xs = self.history.get(X_AXIS)
        if not xs or not isinstance(xs[-1], (int, float)):
            return
        while xs and (xs[0] is None or xs[-1] - xs[0] > HISTORY_SECONDS):
            for d in self.history.values():
                if d:
                    d.popleft()

    # --------------------------------------------------------------- display
    def _update_values_table(self):
        for name, value in self.latest.items():
            if isinstance(value, float):
                text = f"{value:.3f}" if abs(value) < 1e5 else f"{value:.6g}"
            else:
                text = "" if value is None else str(value)
            item = self.value_rows.get(name)
            if item is None:
                parent = self.derived_group if name in DERIVED else self.values_group
                item = self.values.insert(parent, tk.END, text=name)
                self.value_rows[name] = item
            self.values.set(item, "value", text)

    def _update_link_status(self):
        s = self.stats
        self.link_var.set(f"rows {s['rows']}   headers {s['headers']}   "
                          f"lost {s['lost']}   bad lines {s['bad']}")
        if self.last_rx is None:
            self.age_label.configure(text="last packet: -", fg="black")
        else:
            age = time.time() - self.last_rx
            self.age_label.configure(text=f"last packet: {age:.1f} s ago",
                                     fg="#d03b3b" if age > STALE_AFTER_S else "#0ca30c")

    def _redraw_plots(self):
        xs = list(self.history.get(X_AXIS, []))
        if xs:
            for line, name in self.lines:
                ys = self.history.get(name)
                if ys is None:
                    if self.columns and name not in self.warned_missing:
                        self.warned_missing.add(name)
                        self.event(f"plot series '{name}' is not a column or DERIVED value")
                    continue
                pts = [(x, y) for x, y in zip(xs, ys) if isinstance(y, (int, float))]
                line.set_data([p[0] for p in pts], [p[1] for p in pts])
            for ax in self.axes:
                ax.relim()
                ax.autoscale_view()
            self.canvas.draw_idle()
        self.root.after(PLOT_REFRESH_MS, self._redraw_plots)

    def _on_close(self):
        if self.link:
            self._disconnect()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--list", action="store_true", help="list serial ports and exit")
    parser.add_argument("--port", help="port to select at start (or SIMULATED)")
    parser.add_argument("--usb", action="store_true",
                        help="the port is the board's own USB (bench console), not a ground radio")
    parser.add_argument("--flight", action="store_true",
                        help="plot the flight program's columns (FLIGHT_PLOTS), not the bring-up sketch's")
    args = parser.parse_args()
    global USB_DIRECT, PLOTS
    USB_DIRECT = args.usb
    if args.flight:
        PLOTS = FLIGHT_PLOTS
    if args.list:
        for p in list_ports():
            print(p)
        return
    root = tk.Tk()
    app = TelemetryApp(root)
    if args.port:
        app.port_var.set(args.port)
        app._toggle_connect()
    root.mainloop()


if __name__ == "__main__":
    main()
