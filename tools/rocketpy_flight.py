#!/usr/bin/env python3
"""Generate a test flight for the board from a RocketPy 6-DOF simulation.

Flies Voyager (the GNC sim's vehicle model) through a weather forecast, then
writes what the board's sensors would have read, sample by sample, as a log
the board can replay (tools/README.md, "Replay a log on the board"):

    t_us,imu_ok,ax,ay,az,gx,gy,gz,mag_new,mx,my,mz,baro_new,p_pa,temp_c,
    enc_ok,enc0,enc1,enc2,enc3,cmds

  ax..az   specific force (what the accelerometer reads), m/s^2, IMU axes
  gx..gz   body rate, rad/s, IMU axes
  mx..mz   magnetic field, uT (an approximate fixed Earth field, see --mag)
  p_pa     static pressure at the rocket's altitude, from the forecast
  cmds     the arm commands (EnterPreFlight + CanardsEnable) at --arm-at

IMU axes come from include/VehicleConfig.h (kImuToBody, read from the file),
so the data matches how the flight code mounts the IMU.

The flight is OPEN LOOP: RocketPy flies the rocket with its canards fixed at
zero. On the board the canards will move in response, but nothing they do
changes this trajectory. Use it to watch the states, the canard modes and the
telemetry through a whole realistic flight, not to judge control quality.

Needs the GNC sim (Voyager-launch-controller/sim) and its Python, which has
RocketPy 1.13 and netCDF4 (for the forecast):

    SIM=~/Documents/HPRC/EnP/GNC/Voyager-launch-controller/sim
    $SIM/.venv/bin/python tools/rocketpy_flight.py --site cherryfield \\
        --date 2026-11-22 --hour 12 -o canardsLog70.csv

Then copy the CSV to the SD card as /logs/canardsLog<N>.csv and replay it
(tools/usb_stream.py replay <N> --servos --detach).
"""

import argparse
import math
import re
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parent.parent
DEFAULT_SIM = Path.home() / "Documents/HPRC/EnP/GNC/Voyager-launch-controller/sim"

INPUT_HEADER = ("t_us,imu_ok,ax,ay,az,gx,gy,gz,mag_new,mx,my,mz,baro_new,p_pa,temp_c,"
                "enc_ok,enc0,enc1,enc2,enc3,cmds")
ARM_CMDS = "0:-1:0|2:-1:0"  # EnterPreFlight, CanardsEnable (src/Subsystems/Comms/Command.h)
G0 = 9.80665

# Launch sites: latitude, longitude [deg], ground elevation [m], time zone.
SITES = {
    # Town centre; the GFS grid is ~25 km, so the exact field matters little.
    # Elevation is approximate: pass --elevation for the field's real value.
    "cherryfield": (44.607, -67.926, 40.0, "America/New_York"),
}

# Approximate Earth field per site, ENU [uT] (east, north, up). Only heading
# depends on it, which the controller does not use. Cherryfield, ME: total
# ~52 uT, inclination ~67 deg down, declination ~16 deg W (approximate).
SITE_MAG_ENU = {
    "cherryfield": (-5.6, 19.5, -47.9),
}


# --- Flight code constants, read from the source so they stay in sync -------------

def read_imu_to_body():
    text = (REPO / "include/VehicleConfig.h").read_text()
    m = re.search(r"kImuToBody\s*=\s*\{\{(.*?)\}\};", text, re.S)
    if not m:
        raise SystemExit("could not find kImuToBody in include/VehicleConfig.h")
    nums = [float(x.rstrip("f")) for x in re.findall(r"-?\d+\.?\d*f?", m.group(1))]
    if len(nums) != 9:
        raise SystemExit("kImuToBody: expected 9 numbers, found %d" % len(nums))
    return np.array(nums).reshape(3, 3)


def read_flight_config():
    """The few thresholds the expected timeline needs."""
    text = (REPO / "include/FlightConfig.h").read_text()

    def value(name):
        m = re.search(r"constexpr\s+\w+\s+%s\s*=\s*([^;]+);" % name, text)
        if not m:
            raise SystemExit("could not find %s in include/FlightConfig.h" % name)
        expr = m.group(1)
        expr = expr.replace("kGravity", repr(G0)).replace("kDegToRad", repr(math.pi / 180)).replace("kUsPerMs", "1000").replace("kUsPerSecond", "1000000")
        expr = re.sub(r"(\d)f\b", r"\1", expr).replace("ull", "")
        return float(eval(expr, {"__builtins__": {}}))  # numbers and * only

    return {n: value(n) for n in (
        "kLaunchAccel", "kLaunchHoldUs", "kControlStartAltitudeM", "kControlStartHoldUs", "kBoostMaxUs",
        "kControlEndSpeed", "kControlEndHoldUs", "kChuteSpeed", "kChuteHoldUs", "kLandAltitudeM",
        "kLandHoldUs", "kMinControlSpeed", "kMaxTiltRad", "kCutoffHoldUs")}


# --- RocketPy --------------------------------------------------------------------------

def fly(args):
    sys.path.insert(0, str(args.sim))
    from rocketpy import Environment, Flight
    from voyager_sim.plant import build_vehicle, load_yaml

    vehicle = load_yaml(args.sim / "configs/vehicle.yaml")
    lat, lon, elev, tz = SITES[args.site]
    if args.elevation is not None:
        elev = args.elevation
    year, month, day = (int(x) for x in args.date.split("-"))

    env = Environment(latitude=lat, longitude=lon, elevation=elev, date=(year, month, day, args.hour),
                      timezone=tz)
    if args.weather == "forecast":
        print("Fetching the GFS forecast for %s (%.3f, %.3f) at %s %02d:00 %s ..."
              % (args.site, lat, lon, args.date, args.hour, tz), file=sys.stderr)
        env.set_atmospheric_model(type="Forecast", file="GFS")
    else:
        env.set_atmospheric_model(type="standard_atmosphere")
    # A forecast replaces the elevation with the model's ground height. Keep
    # the one asked for, so the pad, the landing and the barometer agree.
    if args.elevation is not None or args.weather == "forecast":
        env.set_elevation(elev)
    elev = float(env.elevation)

    rocket, _, _ = build_vehicle(vehicle, fin_cant_deg=args.fin_cant)

    # Recovery: drogue at apogee, main at --main-alt AGL. Each sized for its
    # descent rate at the dry mass: v = sqrt(2 m g / (rho CdS)).
    dry_mass = vehicle["rocket"]["mass"] + vehicle["motor"]["dry_mass"]

    def cd_s(rate, height_agl):
        rho = float(env.density(elev + height_agl))
        return 2.0 * dry_mass * G0 / (rho * rate ** 2)

    rocket.add_parachute("drogue", cd_s=cd_s(args.drogue_rate, 300.0), trigger="apogee", sampling_rate=100, lag=0.5)
    rocket.add_parachute("main", cd_s=cd_s(args.main_rate, args.main_alt / 2), trigger=float(args.main_alt),
                         sampling_rate=100, lag=0.5)

    flight_kw = dict(vehicle["flight"])
    print("Flying (RocketPy 6-DOF to apogee, then under the chutes) ...", file=sys.stderr)
    flight = Flight(rocket=rocket, environment=env, **flight_kw, terminate_on_apogee=False,
                    max_time=900, max_time_step=0.01, verbose=False)
    return env, flight, elev


def pressure_model(env, elev):
    """Static pressure vs altitude ASL, as the barometer would see it.

    A GFS forecast comes on pressure levels, and RocketPy holds the pressure
    constant below the lowest one (1000 hPa, often 100-200 m above the
    ground). A barometer would see no climb there. Below the lowest level
    that changes, extrapolate hydrostatically with the forecast temperature:
        p(z) = p_ref * exp(-g (z - z_ref) / (R T_ref))."""
    grid = np.arange(elev - 50.0, elev + 3000.0, 5.0)
    p_grid = np.array([float(env.pressure(z)) for z in grid])
    changes = np.where(np.abs(np.diff(p_grid)) > 0.01)[0]
    z_ref = grid[changes[0]] if len(changes) else grid[0]
    p_ref = float(env.pressure(z_ref))
    t_ref = float(env.temperature(z_ref))

    def pressure(z):
        z = np.asarray(z, dtype=float)
        hydro = p_ref * np.exp(-G0 * (z - z_ref) / (287.05 * t_ref))
        above = np.array([float(env.pressure(v)) for v in np.atleast_1d(z)]).reshape(z.shape)
        return np.where(z < z_ref, hydro, above)

    return pressure, z_ref


def series(func, t):
    """A RocketPy Function of time, sampled on t."""
    src = np.asarray(func.source)
    return np.interp(t, src[:, 0], src[:, 1])


def quat_to_matrix(q):
    """Body -> inertial (ENU), Hamilton [w, x, y, z]; rows of q are samples."""
    w, x, y, z = (q[:, i] for i in range(4))
    r = np.empty((len(q), 3, 3))
    r[:, 0, 0] = 1 - 2 * (y * y + z * z)
    r[:, 0, 1] = 2 * (x * y - w * z)
    r[:, 0, 2] = 2 * (x * z + w * y)
    r[:, 1, 0] = 2 * (x * y + w * z)
    r[:, 1, 1] = 1 - 2 * (x * x + z * z)
    r[:, 1, 2] = 2 * (y * z - w * x)
    r[:, 2, 0] = 2 * (x * z - w * y)
    r[:, 2, 1] = 2 * (y * z + w * x)
    r[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return r


# --- Sensor data -------------------------------------------------------------------------

def build_rows(args, env, flight, elev):
    rng = np.random.default_rng(args.seed)
    rate = args.rate
    t_end_flight = float(flight.t_final)
    n_pad = int(round(args.pad * rate))
    n_flight = int(math.floor(t_end_flight * rate)) + 1
    n_ground = int(round(args.ground * rate))
    n = n_pad + n_flight + n_ground
    k = np.arange(n)
    t_board = k / rate                      # board time from the first row, s
    t_sim = np.clip(t_board - args.pad, 0.0, t_end_flight)  # RocketPy time (0 = ignition)

    # Truth from RocketPy, on the board's sample times. Inertial frame: ENU.
    acc = np.column_stack([series(flight.ax, t_sim), series(flight.ay, t_sim), series(flight.az, t_sim)])
    quat = np.column_stack([series(f, t_sim) for f in (flight.e0, flight.e1, flight.e2, flight.e3)])
    quat /= np.linalg.norm(quat, axis=1, keepdims=True)
    omega_r = np.column_stack([series(flight.w1, t_sim), series(flight.w2, t_sim), series(flight.w3, t_sim)])
    z_asl = series(flight.z, t_sim)
    speed = series(flight.speed, t_sim)

    # Before ignition and after landing the rocket is at rest.
    at_rest = (t_board < args.pad) | (t_board - args.pad > t_end_flight)
    acc[at_rest] = 0.0
    omega_r[at_rest] = 0.0
    speed = np.where(at_rest, 0.0, speed)

    # Under the chutes RocketPy flies a point mass and freezes the attitude;
    # its body rates there mean nothing. Hanging under a chute: rates ~0.
    deploy = min((float(p[0]) for p in flight.parachute_events), default=None)
    if deploy is not None:
        under_chute = (t_board - args.pad) >= deploy
        omega_r[under_chute] = 0.0

    rot = quat_to_matrix(quat)              # RocketPy body -> ENU
    f_enu = acc + np.array([0.0, 0.0, G0])  # specific force = a - g, g = (0, 0, -g0)
    f_r = np.einsum("nji,nj->ni", rot, f_enu)  # into RocketPy body (R^T f)
    mag_enu = np.array(SITE_MAG_ENU[args.site])
    m_r = np.einsum("nji,j->ni", rot, mag_enu)

    # RocketPy body (x, y transverse, z = nose) -> team body (x = nose):
    # team (x, y, z) = rocketpy (z, x, y), a cyclic swap, so still right-handed.
    def to_team(v):
        return v[:, [2, 0, 1]]

    imu_to_body = read_imu_to_body()        # body = M * sensor  ->  sensor = M^T * body
    f_s = to_team(f_r) @ imu_to_body
    g_s = to_team(omega_r) @ imu_to_body
    m_s = to_team(m_r)                       # kMagToBody is identity (placeholder): sensor = body

    # Sensor noise and gyro bias (the pad alignment averages the bias out).
    f_s += rng.normal(0.0, args.accel_noise, f_s.shape)
    gyro_bias = rng.normal(0.0, math.radians(args.gyro_bias_dps), 3)
    g_s += gyro_bias + rng.normal(0.0, math.radians(args.gyro_noise_dps), g_s.shape)
    m_s += rng.normal(0.0, args.mag_noise, m_s.shape)

    # Slow sensors: a new sample when their own clock ticks.
    mag_new = np.diff(np.floor(t_board * args.mag_hz), prepend=-1) > 0
    baro_new = np.diff(np.floor(t_board * args.baro_hz), prepend=-1) > 0
    z_for_baro = np.where(t_board < args.pad, elev, z_asl)
    pressure, z_ref = pressure_model(env, elev)
    p = pressure(z_for_baro) + rng.normal(0.0, args.baro_noise, n)
    temp_c = np.array([float(env.temperature(z)) for z in z_for_baro]) - 273.15

    t_us = 1_000_000 + np.round(k * 1e6 / rate).astype(np.int64)
    arm_row = int(round(args.arm_at * rate))

    lines = [INPUT_HEADER]
    for i in range(n):
        lines.append(",".join([
            str(t_us[i]), "1",
            "%.9g" % f_s[i, 0], "%.9g" % f_s[i, 1], "%.9g" % f_s[i, 2],
            "%.9g" % g_s[i, 0], "%.9g" % g_s[i, 1], "%.9g" % g_s[i, 2],
            "1" if mag_new[i] else "0", "%.9g" % m_s[i, 0], "%.9g" % m_s[i, 1], "%.9g" % m_s[i, 2],
            "1" if baro_new[i] else "0", "%.9g" % p[i], "%.9g" % temp_c[i],
            "0", "0", "0", "0", "0",          # no encoder feedback: the servos run open loop
            ARM_CMDS if i == arm_row else "",
        ]))

    truth = dict(t=t_board, agl=z_asl - elev, speed=speed, sf=np.linalg.norm(f_enu, axis=1),
                 pad=args.pad, deploy=deploy, sensor_up=f_s[: max(n_pad, 1)].mean(axis=0),
                 p_ground=float(pressure(elev)), z_ref=z_ref,
                 tilt=np.degrees(np.arccos(np.clip(rot[:, 2, 2], -1.0, 1.0))))
    return lines, truth


# --- What the board should do --------------------------------------------------------------

def held(t, cond, hold_s, start_index=0):
    """First time from start_index at which cond has held for hold_s."""
    run_start = None
    for i in range(start_index, len(t)):
        if cond[i]:
            if run_start is None:
                run_start = t[i]
            if t[i] - run_start >= hold_s - 1e-9:
                return i
        else:
            run_start = None
    return None


def expected_timeline(truth, cfg):
    t, agl, spd, sf = truth["t"], truth["agl"], truth["speed"], truth["sf"]
    out = []
    i = held(t, sf >= cfg["kLaunchAccel"], cfg["kLaunchHoldUs"] / 1e6)
    if i is None:
        return [("launch never detected (specific force never reaches %.1f g)" % (cfg["kLaunchAccel"] / G0), None)]
    out.append(("Boost (launch detected)", t[i]))
    launch = t[i]
    j = held(t, agl > cfg["kControlStartAltitudeM"], cfg["kControlStartHoldUs"] / 1e6, i)
    if j is None or t[j] - launch > cfg["kBoostMaxUs"] / 1e6:
        j = int(np.searchsorted(t, launch + cfg["kBoostMaxUs"] / 1e6))
        out.append(("Control (Boost timeout)", t[j]))
    else:
        out.append(("Control (above %.2f m)" % cfg["kControlStartAltitudeM"], t[j]))
    # The speed exit arms once speed has been above the exit speed in Control.
    above = np.where(spd[j:] > cfg["kControlEndSpeed"])[0]
    start = j + (int(above[0]) if len(above) else 0)
    k = held(t, spd < cfg["kControlEndSpeed"], cfg["kControlEndHoldUs"] / 1e6, start)
    # The envelope cut-off (tilt only here: pitch/yaw rates stay far below
    # their limit in a passive flight) ends Control early if it latches first.
    tilt_limit = math.degrees(cfg["kMaxTiltRad"])
    c = held(t, truth["tilt"] > tilt_limit, cfg["kCutoffHoldUs"] / 1e6, j)
    if c is not None and (k is None or c < k):
        out.append(("Coast (tilt cut-off: tilt over %.0f deg; canards neutral for the rest of the flight)"
                    % tilt_limit, t[c]))
        k = c
    elif k is not None:
        out.append(("Coast (speed under %.0f m/s)" % cfg["kControlEndSpeed"], t[k]))
    if k is not None:
        m = held(t, spd < cfg["kChuteSpeed"], cfg["kChuteHoldUs"] / 1e6, k)
        if m is not None:
            out.append(("Chute (speed under %.0f m/s; servos off)" % cfg["kChuteSpeed"], t[m]))
            apogee_i = int(np.argmax(agl))
            n_ = held(t, (agl < cfg["kLandAltitudeM"]) & (np.arange(len(t)) > apogee_i), cfg["kLandHoldUs"] / 1e6, m)
            if n_ is not None:
                out.append(("Land (under %.0f m AGL)" % cfg["kLandAltitudeM"], t[n_]))
    above_min = np.where(spd >= cfg["kMinControlSpeed"])[0]
    if len(above_min):
        out.append(("(canards may hold attitude while speed >= %.0f m/s, in Control)" % cfg["kMinControlSpeed"], None))
    return out


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--sim", type=Path, default=DEFAULT_SIM, help="the GNC sim folder (default %(default)s)")
    p.add_argument("--site", choices=sorted(SITES), default="cherryfield")
    p.add_argument("--elevation", type=float, help="ground elevation, m ASL (default: the site's)")
    p.add_argument("--date", default="2026-11-22", help="launch date, YYYY-MM-DD (local)")
    p.add_argument("--hour", type=int, default=12, help="launch hour, local time (default 12)")
    p.add_argument("--weather", choices=["forecast", "standard"], default="forecast",
                   help="GFS forecast (needs internet) or the standard atmosphere with no wind")
    p.add_argument("--rate", type=float, default=208.0, help="IMU sample rate, Hz (the board's: 208)")
    p.add_argument("--mag-hz", type=float, default=100.0)
    p.add_argument("--baro-hz", type=float, default=75.0)
    p.add_argument("--pad", type=float, default=60.0, help="seconds on the pad before ignition")
    p.add_argument("--arm-at", type=float, default=5.0, help="when armFlight arrives, s from the first row")
    p.add_argument("--ground", type=float, default=10.0, help="seconds on the ground after landing")
    p.add_argument("--main-alt", type=float, default=150.0, help="main deploy height, m AGL")
    p.add_argument("--drogue-rate", type=float, default=20.0, help="descent rate under the drogue, m/s")
    p.add_argument("--main-rate", type=float, default=6.0, help="descent rate under the main, m/s")
    p.add_argument("--fin-cant", type=float, default=0.0,
                   help="tail-fin cant, deg: makes the rocket roll so the canards have something to fight")
    p.add_argument("--accel-noise", type=float, default=0.05, help="m/s^2, 1 sigma")
    p.add_argument("--gyro-noise-dps", type=float, default=0.3)
    p.add_argument("--gyro-bias-dps", type=float, default=0.1)
    p.add_argument("--mag-noise", type=float, default=0.3, help="uT")
    p.add_argument("--baro-noise", type=float, default=2.0, help="Pa")
    p.add_argument("--seed", type=int, default=1)
    p.add_argument("-o", "--output", required=True, help="the CSV to write (e.g. canardsLog70.csv)")
    args = p.parse_args(argv)
    if args.arm_at >= args.pad:
        raise SystemExit("--arm-at must be before ignition (--pad)")

    env, flight, elev = fly(args)
    lines, truth = build_rows(args, env, flight, elev)
    Path(args.output).write_text("\n".join(lines) + "\n")

    # Sanity: on the pad the accelerometer must read +1 g along the nose.
    nose_sensor = read_imu_to_body()[0]     # body x in sensor axes is row 0 of M (body = M * sensor)
    up = float(np.dot(truth["sensor_up"], nose_sensor)) / G0
    pad = truth["pad"]
    apogee_i = int(np.argmax(truth["agl"]))
    print("\nwrote %s: %d rows, %.1f s at %.0f Hz (%.1f MB)"
          % (args.output, len(lines) - 1, truth["t"][-1], args.rate, Path(args.output).stat().st_size / 1e6))
    print("weather: %s; ground wind %.1f m/s (%.0f mph) FROM %.0f deg, %.1f C, %.0f Pa at the pad"
          % (args.weather, float(env.wind_speed(elev + 2)), float(env.wind_speed(elev + 2)) * 2.23694,
             float(env.wind_direction(elev + 2)), float(env.temperature(elev)) - 273.15, truth["p_ground"]))
    if truth["z_ref"] > elev:
        print("         (forecast pressure starts %.0f m above the pad; extrapolated below that)"
              % (truth["z_ref"] - elev))
    print("flight:  apogee %.0f m AGL at %.1f s, max speed %.0f m/s, rail exit %.2f s, lands %.1f s after ignition"
          % (truth["agl"][apogee_i], truth["t"][apogee_i] - pad, float(np.max(truth["speed"])),
             float(flight.out_of_rail_time), float(flight.t_final)))
    in_flight = truth["speed"] > 0
    print("tilt:    %.1f deg at apogee; max %.1f deg while above 30 m/s"
          % (truth["tilt"][apogee_i], float(np.max(truth["tilt"][truth["speed"] >= 30.0])) if np.any(truth["speed"] >= 30.0) else 0.0))
    print("check:   on the pad the IMU reads %+.3f g along the nose (must be +1.000)" % up)

    cfg = read_flight_config()
    print("\nexpected on the board (s from the first row; ignition at %.1f s):" % pad)
    print("  %7.2f  PreFlight (armFlight in the data)" % args.arm_at)
    for name, when in expected_timeline(truth, cfg):
        print(("  %7.2f  %s" % (when, name)) if when is not None else ("           %s" % name))
    print("\nOPEN LOOP: the canards respond on the board but don't change this trajectory.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
