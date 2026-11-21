#!/usr/bin/env python3
"""Read the canard flight log (src/Subsystems/Logger/CycleLogFormat.h).

  python3 tools/flight_log.py summary LOG.csv
      states with their entry times, canard modes, safety bits, dropped rows

  python3 tools/flight_log.py compare A.csv B.csv
      aligns rows by t_us and reports the first row where state, mode,
      cmd0..3 or pulse0..3 differ, plus counts. Exit status 1 if any differ.

  python3 tools/flight_log.py inputs LOG.csv [-o OUT.csv]
      only the input columns (what replay and the USB stream read)

Columns are found by name from the header, as the format requires.

SD replay and log streaming are OPEN LOOP: comparing a replay with the
original shows whether the new code changes state at the same moments and
commands sane values, NOT whether it would have flown straight.

Standard library only.
"""

import argparse
import csv
import sys

INPUT_COLUMNS = [
    "t_us", "imu_ok", "ax", "ay", "az", "gx", "gy", "gz",
    "mag_new", "mx", "my", "mz", "baro_new", "p_pa", "temp_c",
    "enc_ok", "enc0", "enc1", "enc2", "enc3", "cmds",
]
COMPARED_COLUMNS = ["state", "mode"] + ["cmd%d" % i for i in range(4)] + ["pulse%d" % i for i in range(4)]

# src/StateMachine/StateId.h
STATES = ["Init", "PreFlight", "Boost", "Control", "Coast", "Chute", "Land"]
# src/Subsystems/Canards/Canards.h, CanardMode
MODES = ["Off", "Neutral", "Controlling", "Testing"]
# src/Safety/SafetyMonitor.h, SafetyFault
SAFETY_BITS = ["ImuGap", "TimeBackwards", "ImuReadFailed", "ImuNonFinite", "TransitionRefused"]
# src/Subsystems/Canards/Canards.h, CanardGate
GATE_BITS = ["NotEnabled", "LockedOut", "SafetyFault", "NavInvalid", "SpeedLow", "Cutoff",
             "ControllerFailed", "NonFinite"]
# src/Subsystems/Comms/Command.h, CommandType
COMMANDS = ["EnterPreFlight", "StandDown", "CanardsEnable", "CanardsDisable", "CanardTest",
            "CanardTestStop", "CanardSweep"]

OPEN_LOOP_NOTE = ("NOTE: replay and streaming are open loop. Matching outputs show the new code "
                  "changes state at the same moments and commands the same values; they do not "
                  "show whether it would have flown straight.")


def name_of(names, value):
    try:
        i = int(value)
    except ValueError:
        return "?%s" % value
    return names[i] if 0 <= i < len(names) else "?%d" % i


def bit_names(names, value):
    v = int(value)
    out = [n for i, n in enumerate(names) if v & (1 << i)]
    rest = v & ~((1 << len(names)) - 1)
    if rest:
        out.append("0x%x" % rest)
    return out


def read_log(path):
    """Returns (header, rows), rows as dicts. Skips blank and '#' lines, and
    rows whose t_us is not an integer (counted in the second value)."""
    with open(path, newline="") as f:
        lines = [ln for ln in f if ln.strip() and not ln.lstrip().startswith("#")]
    reader = csv.reader(lines)
    try:
        header = next(reader)
    except StopIteration:
        raise SystemExit("%s: empty log" % path)
    if "t_us" not in header:
        raise SystemExit("%s: no t_us column in the header; not a flight log?" % path)
    rows, bad = [], 0
    for fields in reader:
        if len(fields) < len(header) or not fields[header.index("t_us")].isdigit():
            bad += 1
            continue
        rows.append(dict(zip(header, fields)))
    return header, rows, bad


def seconds(t_us, t0):
    return (int(t_us) - t0) / 1e6


# --- summary ---------------------------------------------------------------------

def summary(path):
    header, rows, bad = read_log(path)
    print("%s: %d rows%s" % (path, len(rows), (", %d malformed skipped" % bad) if bad else ""))
    if not rows:
        return 0
    t0 = int(rows[0]["t_us"])
    t1 = int(rows[-1]["t_us"])
    print("time: t_us %d .. %d (%.3f s)" % (t0, t1, (t1 - t0) / 1e6))
    have = set(header)

    if "state" in have:
        print("\nstates (entry time, s from the first row):")
        prev = None
        for r in rows:
            if r["state"] != prev:
                print("  %8.3f  %s" % (seconds(r["t_us"], t0), name_of(STATES, r["state"])))
                prev = r["state"]
        refused = rows[-1].get("refused", "0")
        if refused not in ("", "0"):
            print("  refused transitions: %s" % refused)

    if "mode" in have:
        print("\ncanard modes:")
        prev = None
        for r in rows:
            key = (r["mode"], r.get("enabled"), r.get("locked"), r.get("cutoff"))
            if key != prev:
                flags = [n for n, c in (("enabled", "enabled"), ("locked out", "locked"), ("cut-off", "cutoff"))
                         if r.get(c) == "1"]
                print("  %8.3f  %-11s %s" % (seconds(r["t_us"], t0), name_of(MODES, r["mode"]), " ".join(flags)))
                prev = key
        if "denied" in have:
            print("  denied requests: %s" % rows[-1]["denied"])

    if "safety" in have:
        print("\nsafety faults (first seen, s; cycles active):")
        first, count = {}, {}
        for r in rows:
            for n in bit_names(SAFETY_BITS, r["safety"] or "0"):
                first.setdefault(n, seconds(r["t_us"], t0))
                count[n] = count.get(n, 0) + 1
        if not first:
            print("  none")
        for n in sorted(first, key=first.get):
            print("  %8.3f  %-18s %d" % (first[n], n, count[n]))
        if "safety_sticky" in have:
            print("  sticky at end: %s" % (", ".join(bit_names(SAFETY_BITS, rows[-1]["safety_sticky"] or "0"))
                                          or "none"))

    if "cmds" in have:
        n_cmds = 0
        print("\nground commands:")
        for r in rows:
            if r["cmds"]:
                items = r["cmds"].split("|")
                n_cmds += len(items)
                names = []
                for it in items:
                    parts = it.split(":")
                    names.append(name_of(COMMANDS, parts[0]) +
                                 ("(%s,%s)" % (parts[1], parts[2]) if parts[0] == "4" and len(parts) == 3 else ""))
                print("  %8.3f  %s" % (seconds(r["t_us"], t0), " ".join(names)))
        if not n_cmds:
            print("  none")

    print("\ndrops:")
    if "cycle" in have:
        missing, gaps = 0, 0
        prev = None
        for r in rows:
            c = int(r["cycle"])
            if prev is not None and c > prev + 1:
                missing += c - prev - 1
                gaps += 1
            prev = c
        print("  log rows missing (cycle numbers skipped): %d in %d gaps" % (missing, gaps))
    periods = sorted(int(b["t_us"]) - int(a["t_us"]) for a, b in zip(rows, rows[1:]))
    if periods:
        median = periods[len(periods) // 2]
        long_gaps = sum(1 for p in periods if p > 2 * median)
        print("  sample period: median %d us, max %d us, %d gaps over twice the median"
              % (median, periods[-1], long_gaps))
    return 0


# --- compare ---------------------------------------------------------------------

def _value(row, col):
    v = row.get(col)
    if v is None:
        return None
    try:
        return float(v)
    except ValueError:
        return v


def _differs(a, b, col, tol):
    va, vb = _value(a, col), _value(b, col)
    if isinstance(va, float) and isinstance(vb, float):
        return abs(va - vb) > tol
    return va != vb


def compare(path_a, path_b, tol):
    ha, ra, _ = read_log(path_a)
    hb, rb, _ = read_log(path_b)
    cols = [c for c in COMPARED_COLUMNS if c in ha and c in hb]
    missing = [c for c in COMPARED_COLUMNS if c not in cols]
    if missing:
        print("not in both logs, not compared: %s" % ", ".join(missing))

    # Align by t_us. Repeated times (a log whose clock went backwards) are
    # paired in order of appearance.
    def index(rows):
        out = {}
        for r in rows:
            out.setdefault(int(r["t_us"]), []).append(r)
        return out

    ia, ib = index(ra), index(rb)
    times = sorted(set(ia) & set(ib))
    only_a = sum(len(v) for t, v in ia.items() if t not in ib)
    only_b = sum(len(v) for t, v in ib.items() if t not in ia)

    first = None
    differing = 0
    per_col = {c: 0 for c in cols}
    pairs = 0
    for t in times:
        for a, b in zip(ia[t], ib[t]):
            pairs += 1
            row_differs = False
            for c in cols:
                if _differs(a, b, c, tol):
                    per_col[c] += 1
                    row_differs = True
                    if first is None:
                        first = (t, a, b)
            differing += row_differs
        only_a += max(0, len(ia[t]) - len(ib[t]))
        only_b += max(0, len(ib[t]) - len(ia[t]))

    print("A: %s (%d rows)" % (path_a, len(ra)))
    print("B: %s (%d rows)" % (path_b, len(rb)))
    print("aligned by t_us: %d rows in both, %d only in A, %d only in B" % (pairs, only_a, only_b))
    print("rows differing in %s: %d" % ("/".join(cols), differing))
    for c in cols:
        if per_col[c]:
            print("  %-8s %d" % (c, per_col[c]))
    if first is not None:
        t, a, b = first
        t0 = int(ra[0]["t_us"])
        print("first difference at t_us %d (%.3f s into A, cycle A %s / B %s):"
              % (t, (t - t0) / 1e6, a.get("cycle", "?"), b.get("cycle", "?")))
        for c in cols:
            va, vb = a.get(c), b.get(c)
            mark = "  <-" if _differs(a, b, c, tol) else ""
            if c == "state":
                va, vb = "%s(%s)" % (name_of(STATES, va), va), "%s(%s)" % (name_of(STATES, vb), vb)
            elif c == "mode":
                va, vb = "%s(%s)" % (name_of(MODES, va), va), "%s(%s)" % (name_of(MODES, vb), vb)
            print("  %-8s A %-16s B %-16s%s" % (c, va, vb, mark))
    else:
        print("no differences")
    print(OPEN_LOOP_NOTE)
    return 1 if (first is not None) else 0


# --- inputs ----------------------------------------------------------------------

def inputs(path, out_path):
    header, rows, bad = read_log(path)
    missing = [c for c in INPUT_COLUMNS if c not in header]
    if missing:
        raise SystemExit("%s: missing input columns: %s" % (path, ", ".join(missing)))
    out = open(out_path, "w", newline="") if out_path else sys.stdout
    try:
        w = csv.writer(out, lineterminator="\n")
        w.writerow(INPUT_COLUMNS)
        for r in rows:
            w.writerow([r[c] for c in INPUT_COLUMNS])
    finally:
        if out_path:
            out.close()
    if bad:
        print("%d malformed rows skipped" % bad, file=sys.stderr)
    return 0


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    s = sub.add_parser("summary", help="states, modes, safety bits, drops")
    s.add_argument("log")
    c = sub.add_parser("compare", help="first differing row of two logs, aligned by t_us")
    c.add_argument("a")
    c.add_argument("b")
    c.add_argument("--tol", type=float, default=0.0,
                   help="allowed difference for numeric columns (default 0: identical)")
    i = sub.add_parser("inputs", help="only the input columns")
    i.add_argument("log")
    i.add_argument("-o", "--output", help="write here instead of stdout")
    args = p.parse_args(argv)
    if args.cmd == "summary":
        return summary(args.log)
    if args.cmd == "compare":
        return compare(args.a, args.b, args.tol)
    return inputs(args.log, args.output)


if __name__ == "__main__":
    sys.exit(main())
