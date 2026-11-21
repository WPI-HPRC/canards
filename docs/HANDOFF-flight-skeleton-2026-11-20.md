# Canards flight software: skeleton handoff (2026-11-20)

This hands off the design of the canard rocket's flight program to the agent
that will build it. Everything below was discussed and agreed with the user
unless marked **proposal**. A proposal is a recommendation that has not been
confirmed; build it that way, but list it for the user to confirm.

**Goal of this handoff:** build the skeleton of the flight program in the
`canards` repo, meaning the structure, the state machine, the canard subsystem
and the interfaces, with desktop tests. Then port the tested pieces from the
older repos into it, in the order given in section 10.

**Working rules:**
- Do not commit or push unless the user asks.
- Do not edit the estimator. Another person owns it (section 2).
- Do not pick safety-critical numbers yourself. Use a tagged placeholder and
  ask (section 9).
- The user wants design discussion before code on anything not covered here.
  If a design question comes up that this doc does not answer, ask; don't
  decide silently.

---

## 1. Scope and ownership

| Area | Owner | What we do |
|---|---|---|
| State machine, flight sequencing | **us** | build it |
| Attitude controller (LQR) and servo PID | **us** | port and integrate |
| Hardware tests (move canards from the ground station or bench) | **us** | build |
| Lockouts and safety checks | **us** | build |
| Sim on the board (USB stream) and SD flight replay | **us** | build |
| State estimator | someone else, actively writing it | define and keep the interface only |
| Ground station and radio protocol | someone else | define the command interface only |
| Sensor and servo drivers | existing code | wrap behind interfaces; not our focus |

This is an experimental flight, not a competition. The user will supply exact
rules and limits later.

## 2. The repos

| Repo | Path | Role |
|---|---|---|
| **canards** (WPI-HPRC/canards), **the target** | `~/Documents/HPRC/Voyager/canards` | the new flight program goes here |
| Canards-Rocket | `~/Documents/HPRC/Voyager/Canards-Rocket` (branch `feat/canards-state`) | reference only: tested canard code, bench firmware, hardware facts |
| Voyager-launch-controller | `~/Documents/HPRC/EnP/GNC/Voyager-launch-controller` (no commits) | reference: **newest** controller copy, plus the RocketPy 6-DOF sim |
| SlidingModeLQRSim | `~/Documents/HPRC/EnP/GNC/SlidingModeLQRSim` | older Python-only sim; origin of `vehicle.yaml` |
| pid-tuner | `~/Documents/HPRC/Voyager/pid-tuner` | servo PID tuner firmware (stale pins; see the old handoff) |

Earlier handoff, with hardware facts and calibration:
`~/Documents/HPRC/EnP/GNC/HANDOFF-canards-2026-11-20.md`.

**State of the canards repo (2026-11-20):**
- Branches are `master` and `navigator-init`. The main checkout is on
  `navigator-init`.
- Commit `b4bd140` ("navigator adding to main", by Nic DeBruin) does three
  things:
  - adds `src/simple_estimator.{h,cpp}`;
  - adds BasicLinearAlgebra to `lib_deps`;
  - puts an estimator test loop into `src/main.cpp`.

  That work is active, so expect conflicts on `src/main.cpp` and
  `platformio.ini`.
- `src/main.cpp` is a bring-up sketch: it initialises the sensors, GPS and SD
  card, has the radio commented out, and prints debug output. `sdcard.*`,
  `max10s.*` and `dfu_bootloader.cpp` sit in `src/`. `lib/PwmInput` reads the
  encoders. `lib/LoRaE22` is the radio, a submodule.
- Submodules: run `git submodule update --init board lib/LoRaE22` in a fresh
  checkout. `lib/flatbuffers` is a gitlink with no `.gitmodules` entry. Leave
  it alone, and tell the user.
- A worktree may exist at `.claude/worktrees/flight-skeleton` (branch
  `flight-skeleton`). It holds a partial start I made before the user said
  this step is documentation only: a moved `bringup/main.cpp`,
  `platformio.ini` envs, `lib/Frames`, two interface headers and two config
  headers. Check with the user whether it still exists and whether to reuse
  it. Do not assume it is reviewed.

**Branching:** work on a new branch off the user's chosen base. Ask whether
that is `navigator-init` or `master`. Keep the bring-up program buildable as
its own env (section 3) instead of deleting it.

## 3. Board and build facts

**The microcontroller:**
- STM32H753ZIT6 (Cortex-M7), with a double-precision FPU (`-mfpu=fpv5-d16`).
- **240 MHz** system clock: 25 MHz crystal, PLL M=5, N=96, P=2. This is set in
  `board/variants/MARSv21/variant_MARSV21.cpp:152`. AHB runs at 120 MHz and APB
  at 60 MHz.
- 2 MB flash. The board file lists 512 KB of RAM.
- Arduino core (stm32duino), `platform = ststm32@19.6.0`. There is no RTOS.
  Keep it that way: threads would break deterministic replay.

**Tools:**
- PlatformIO is at `~/.platformio/penv/bin/pio`, which is not on PATH.

**Upload:**
- Upload is DFU, entered with a 1200-baud USB touch: `scripts/dfu_touch.py` on
  the host and `src/dfu_bootloader.cpp` on the board.
- The touch only works while USB CDC is running. Every program's `setup()` must
  call `SerialUSB.begin()`, or reflashing needs the BOOT0 button.
- If the board stays solid blue after an upload, press reset.

**Timing facts. Read these before choosing a control rate:**
- **"200 Hz" is a design choice in the sims, not a hardware property.** It's set
  in `SlidingModeLQRSim/configs/control.yaml:15` and
  `Voyager-launch-controller/sim/configs/sim.yaml:5` ("the gains assume it").
  The LQR gains are computed for that step (`dt` = 5 ms). Nobody has measured
  the firmware loop rate.
- **The servos only take a new pulse every 20 ms (50 Hz PWM).** A 200 Hz
  controller therefore really actuates at 50 Hz, and the controller design does
  not model that extra delay. Ask the user whether the RS100 accepts a faster
  frame rate; many digital servos take 333 Hz.
- **IMU output rates differ between the codebases:**
  - The canards bring-up sets the LSM6 gyro to **100 Hz** and the accelerometer
    to 500 Hz (`bringup`/`main.cpp`, around line 256).
  - Canards-Rocket's boilerplate uses 208 Hz.

  The design below runs one cycle per IMU sample, so the gyro rate *is* the
  control rate. Pick one rate, use it for both the gyro and the accelerometer,
  and make sure the LQR gains are computed for exactly that `dt`. Ask the user
  before choosing.
- **The SD card uses software SPI** (`-DSPI_DRIVER_SELECT=2`) and can stall for
  milliseconds. Logging must buffer and write in chunks, never inside the
  control step. Canards-Rocket's `src/logging.cpp` is a starting point.

**Build envs to end up with:**
- `marsv21`: the flight program (default). `marsv21_debug` is the same with
  `-D DEBUG`.
- `marsv21_bringup` (and `_debug`): the existing bring-up program, moved to
  `bringup/main.cpp` and built with a `build_src_filter`.
- `native`: desktop tests (`pio test -e native`). Use `test_build_src = yes`
  and a `build_src_filter` that lists only the hardware-free folders of `src/`.
  Canards-Rocket's `[env:test_desktop]` (`platformio.ini:124`) is the pattern.
  The filter enforces that flight logic never includes Arduino headers.
- Later: `marsv21_bench` for the ground-test program (section 7).

The old handoff says `pio test -e native` for Canards-Rocket; there the env is
actually `test_desktop`.

## 4. Conventions

**Frames (decided):**
- Inertial frame: NED.
- Body frame: **x out of the nose**, then y and z, with z pointing down when the
  rocket lies on its side.
- Body rates: [x] roll, [y] pitch, [z] yaw.
- **Quaternions:** Hamilton convention, stored `[w, x, y, z]`, unit length. They
  rotate body vectors into NED: `v_ned = q v_body q*`. Put this in one header
  (`lib/Frames`) and agree it with the estimator owner.

**Gimbal lock (decided: no Euler angles in flight logic):** flying nose-up in
this frame is pitch = +90°, which is exactly where yaw-pitch-roll angles are
singular. So:
- **Tilt** is `acos(dot(R(q)·x̂_body, up_ned))`, with `up_ned = (0,0,−1)`.
- **Attitude error** is the rotation vector `Log(R_refᵀ R)`, i.e. the log of
  `conj(q_ref) ⊗ q`. Take the shorter of q and −q (w ≥ 0). It is only singular
  at a 180° error.
- **The vertical reference** is the smallest rotation that takes the nose to up,
  applied on the left in NED: `normalize([1 + n·u, n × u]) ⊗ q`. It keeps the
  current roll. If the rocket points almost straight down, return q unchanged.
- **The roll angle about the nose,** if a roll hold ever needs one, comes from a
  swing-twist decomposition, never from Euler roll.
- **Euler angles** are allowed only for display and log decoding.

**Units:** SI everywhere: m, m/s, rad, rad/s, Pa, µT.

**Time:**
- The **only** clock in flight logic is the IMU sample time in the sensor frame,
  as a `uint64_t` in µs. A `uint32_t` µs counter wraps after 71 minutes, and the
  rocket can sit on the pad longer than that.
- **No `millis()` or `micros()` anywhere downstream of the sensor source.** That
  rule is what makes replay deterministic.

**Memory:**
- No heap allocation after `setup()`.
- States and subsystems are long-lived objects. Do not use Canards-Rocket's
  `malloc`/`free` per state.

## 5. Directory layout (decided)

```
canards/
  platformio.ini        envs: marsv21 (flight), marsv21_bringup, native, later marsv21_bench
  include/              config headers, no Arduino
    FlightConfig.h      transition thresholds, canard limits, safety limits
    VehicleConfig.h     sensor mount maps, servo calibration, canard->channel map
  src/
    main.cpp            a few lines: build the FlightComputer, call cycle() in loop()
    FlightComputer.*    owns everything and runs the fixed cycle (section 6.2)
    StateMachine/       State interface, StateMachine runner, allowed transitions,
                        Persistence (debounce), states/ (one .h/.cpp per state)
    Subsystems/
      Canards/          the canard subsystem (section 6.4): the only way states touch canards
      Navigation/       wraps the Estimator: axis maps, |accel|, freshness, speed, tilt
      Logger/           log sink interface + cycle record
      Comms/            Command types + command source interface (ground station boundary)
    Estimator/          Estimator interface + NullEstimator; the estimator owner's code plugs in here
    AttitudeControl/    LQR port (section 8.1)
    ServoControl/       canard angle -> pulse, calibration, per-servo PID (section 8.2)
    Safety/             system-wide monitor (IMU gaps, time going backwards, etc.)
  lib/                  shared, general-purpose code
    Frames/             Vec3, Quat, rotations, attitude error, tilt, vertical reference
    SensorSource/       SensorFrame + SensorSource interface; Live / UsbStream / SdReplay
    Actuators/          ActuatorFeedback/Command + Actuators interface; MARS implementation
    FlightLog/          binary record formats shared with tools/
    PwmInput/ LoRaE22/  (existing)
  bringup/              the existing bring-up program
  bench/                ground-test program (later)
  test/                 desktop tests, one folder per suite
  tools/                Python: log decoder, sim streamer, replay host
```

The user chose to put StateMachine, Estimator, AttitudeControl and ServoControl
in `src/`. Safety and Subsystems follow the same rule.

**Arduino rules:**
- Only `lib/SensorSource`, `lib/Actuators`, `src/main.cpp`, `bringup/` and
  `bench/` may touch Arduino.
- Guard any Arduino-only `.cpp` inside a `lib/` folder with
  `#if defined(ARDUINO)`. That way the `native` env can still use the library's
  headers.

## 6. The flight program

### 6.1 States and transitions (decided)

`Init → PreFlight → Boost → Control → Coast → Chute → Land`

| State | Canards | Leaves when | To |
|---|---|---|---|
| **Init** | neutral; ground tests allowed | the ground station sends "go to preflight" | PreFlight |
| | | **or** launch detected (5 g, debounced) | Boost, with canards **locked out for the whole flight** |
| **PreFlight** | neutral; ground tests allowed | launch detected: \|specific force\| ≥ 5 g, held `kLaunchHoldUs` | Boost |
| | | a stand-down command (**proposal**, for a scrub) | Init |
| **Boost** | **neutral, servos powered** | altitude ≥ 50 ft (15.24 m) AGL, held; **or** `kBoostMaxUs` in the state ("x seconds", placeholder) | Control |
| **Control** | LQR attitude hold | speed < `kControlEndSpeed` held; **or** the envelope cut-off latched; **or** `kControlMaxUs` timeout | Coast |
| **Coast** | neutral, powered | speed < ~30 m/s near apogee, held; **or** timeout | Chute |
| **Chute** | **servos powered off** | altitude < `kLandAltitudeM` ("nearing the ground"), held; **or** timeout | Land |
| **Land** | powered off | never (terminal) | — |

More on individual states:
- **Init** is entered at power-on, and the board is powered on the pad. Its
  hardware bring-up is fast, but the state lasts until about a minute before
  launch.
- **PreFlight `init()`** calls `estimator.reinitialize()`: gyro bias, pad
  alignment and altitude reference. **Proposal:** a repeated "go to preflight"
  command while already in PreFlight re-initialises again.
- **Every state has a timeout,** so a failed detection cannot strand the flight.
  The descent timeouts only move us to the next state, and logging continues in
  every state, so an early Land loses nothing.
- **Launch detection uses raw \|specific force\| from the IMU,** never the
  estimator.

**Mission profile (user):** the canards control the vehicle above the speed at
which they have authority, and stop below a threshold shortly before apogee.
There are no pyro charges on this rocket, so the board fires nothing in Chute
or Land.

**Allowed transitions** are declared once, as a list of from → to pairs:
- Init → PreFlight
- Init → Boost
- PreFlight → Init (proposal)
- PreFlight → Boost
- Boost → Control
- Control → Coast
- Coast → Chute
- Chute → Land

If a state's `next()` returns a pair that isn't listed, the state machine
refuses it: it stays in the current state, counts the refusal and logs it.

### 6.2 State lifecycle and the cycle (decided)

Each state has four methods. `next()` replaces the bool `isFinished()` the user
first proposed, so that states can branch.

```cpp
class State {
 public:
  virtual void    init(const Cycle&)    {}   // once, on entry
  virtual void    execute(const Cycle&) {}   // every cycle while active
  virtual void    end(const Cycle&)     {}   // once, right after the last execute()
  virtual StateId next(const Cycle&) = 0;    // where to go; own id = stay
};

struct Cycle {                 // what a state may read this cycle
  uint64_t timeUs;             // IMU sample time
  float    dt;                 // s since previous sample
  uint64_t timeInStateUs;      // filled in by the state machine
  const Navigation& nav;
  const CommandList& commands; // ground commands received this cycle
};
```

**Order within one cycle:**
1. `execute()`
2. `next()`
3. If `next()` returned an allowed transition: the old state's `end()`, then the
   new state's `init()`, in the same cycle.
4. The new state's first `execute()` runs on the **next** cycle.

On the very first cycle, the state machine calls `Init.init()` first.

`end()` takes no `interrupted` flag (**proposal**: nothing interrupts a state in
this design, because safety is separate from the states). Add the flag only if
a forced transition is ever introduced.

**`FlightComputer::cycle()` is the only thing `loop()` calls.** It does nothing
until the sensor source has a new IMU sample. Then it runs, in this order:
1. Read the sensor frame. Compute `dt`, and detect time going backwards.
2. `Navigation.update(frame, onPad)`: apply the mount maps, run the estimator,
   and derive \|accel\|, speed, tilt and freshness.
3. Drain the ground commands into a fixed-size `CommandList`, at most N per
   cycle; any extra wait for the next cycle. Log each command. Send
   canard enable and disable commands to `Canards::handleCommand()`.
4. `SafetyMonitor.check(...)`.
5. `Canards.beginCycle()`, which clears last cycle's request.
6. `StateMachine.step(cycle)`.
7. Read the actuator feedback (encoders).
8. `Canards.periodic(cycle, currentState, safetyActive, feedback, out)`.
9. `Actuators.write(out)`.
10. Log the cycle record.

**`main.cpp` stays tiny.** It constructs the concrete sources and sinks (live,
USB stream or SD replay, chosen at boot) and the `FlightComputer`, calls
`SerialUSB.begin()` in `setup()`, and calls `flight.cycle()` in `loop()`.

**Helpers:**
- `Persistence`: a condition must hold continuously for T µs of *sample time*.
  Use it for every transition condition.
- One `GroundTest` helper, shared by Init and PreFlight. It turns test commands
  into a held set of test angles that expires after `kGroundTestHoldUs`, and
  requests those angles every cycle while active.

### 6.3 Why a state machine plus subsystems (decided)

The pattern follows WPILib's command-based design: states are the commands,
and the canards are a subsystem. Alternatives were considered and rejected:
- **A transition table:** it splits a state's logic between the state and the
  table, which isn't worth it for 7 states. The allowed-transitions list keeps
  the review benefit.
- **A hierarchical state machine:** the only transition that applies everywhere
  is "safe the canards", and the safety latch already covers it.
- **RTOS tasks:** they break deterministic replay.

### 6.4 The Canards subsystem (decided)

States never touch the controller, servos or hardware directly. They make
**requests**:

```cpp
canards.requestNeutral();
canards.requestAttitudeHold(referenceQuat);  // LQR
canards.requestTestAngles(rad[4]);           // ground test
```

**A request lasts one cycle.** If nothing asks in a cycle, the result is
neutral, so a forgotten branch or a state change can never leave the canards on
a stale command. **Proposal:** two different requests in the same cycle also
result in neutral.

**Permission table, checked in `periodic()` against the current state.** A
request that isn't permitted is treated as neutral, counted and logged.

| Request | Init | PreFlight | Boost | Control | Coast | Chute | Land |
|---|---|---|---|---|---|---|---|
| neutral | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ | ✓ |
| test angles | ✓ | ✓ | ✗ | ✗ | ✗ | ✗ | ✗ |
| attitude hold | ✗ | ✗ | ✗ | ✓ | ✗ | ✗ | ✗ |
| **servo power** | if enabled | if enabled | if enabled | if enabled | if enabled | **off** | **off** |

The test-angle rule means a ground test can never move a canard after launch is
detected. The state used for the check is the one in force *after* this cycle's
transition. A test requested in the same cycle that launch is detected is
therefore refused.

**Rules enforced in `periodic()`, every cycle, whatever the state:**
- **Enable and disable:**
  - Canards start **disabled** at boot.
  - `CanardsEnable` is accepted only in Init or PreFlight. `CanardsDisable` is
    accepted always.
  - Disabled means servo power off.
  - **Proposal**, from the open "failure behaviour" question: whether disabling
    in flight should hold neutral with power on instead. Ask the user.
- **Locked out if launch skipped PreFlight:** if the rocket reaches any flight
  state without having been in PreFlight, attitude hold is refused for the rest
  of the flight. This is the user's "follow the flight, no canards" decision.
- **Gates on attitude hold.** The result is neutral if any of these is true:
  - not enabled;
  - locked out;
  - a system safety fault is active;
  - navigation is invalid or older than `kNavStaleUs`;
  - speed < `kMinControlSpeed`;
  - the controller reports failure;
  - any output is not a finite number.
- **Envelope cut-off:** tilt > `kMaxTiltRad`, or the pitch or yaw rate exceeds
  `kMaxPitchYawRate`, latches neutral for the rest of the flight. The status
  exposes the latch so `Control.next()` can move to Coast.
- **Limits:** attitude-hold commands are clamped to `kCanardLimitRad` (4°), and
  test angles to `kGroundTestLimitRad` (7.5°, the linkage failsafe). Pulses are
  clamped to the calibrated limits. An uncalibrated channel only ever gets its
  centre pulse.
- **Status** for logs, telemetry and `next()`:
  - mode: off / neutral / controlling / testing;
  - enabled, powered, locked out, cut-off latched;
  - denied-request count and the last denied request;
  - the commanded canard angles.

**SafetyMonitor:** checks that span the whole system: IMU gaps over
`kMaxImuGapUs`, time going backwards, non-finite IMU values, refused
transitions. Each fault has an *active* bit, which forces the canards to
neutral that cycle, and a *sticky* bit, kept for the logs. **Proposal:**
transient faults are not latched for the whole flight, because one SD stall
should not kill control. Ask the user. The hardware watchdog (IWatchdog) is
started from `main.cpp`.

### 6.5 Interfaces to other owners

**Estimator** (`src/Estimator/Estimator.h`). The estimator owner implements
this; we only consume `NavState`.

```cpp
struct EstimatorInput { uint64_t timeUs; float dt; bool onPad;
  Vec3 accelBody, gyroBody; bool magFresh; Vec3 magBody; bool baroFresh; float pressurePa; };
struct NavState { uint64_t timeUs; Quat attitude /*body->NED*/; Vec3 bodyRate;
  Vec3 velocityNed; float altitudeAgl; bool attitudeValid, velocityValid, altitudeValid; };
class Estimator { virtual void reinitialize() = 0;
                  virtual void update(const EstimatorInput&, NavState&) = 0; };
```

Notes on the estimator:
- `NullEstimator` reports nothing valid. With it, the canards never control and
  every navigation-based transition falls back to its timeout. That is the safe
  default until the real estimator lands.
- `src/simple_estimator.*` on `navigator-init` is the owner's current work. It
  uses BasicLinearAlgebra and is commented as returning roll, pitch and yaw.
  Wrap it behind the interface; don't modify it. Agree the frame and the
  quaternion convention with its owner.
- **Speed:** the controller schedules its gains on speed, and the sim feeds it
  *inertial* speed, not airspeed (`flight.py:209` in launch-controller). Ask the
  estimator owner what speed they can provide. Flag to the user that wind makes
  the two differ.

**Ground station** (`src/Subsystems/Comms/`). The other owner maps radio packets
to our `Command` struct and implements a `CommandSource`:

```cpp
enum class CommandType : uint8_t { EnterPreFlight, StandDown, CanardsEnable, CanardsDisable,
                                   CanardTest, CanardTestStop };
struct Command { CommandType type; int8_t canard /* -1 = all */; float angleRad; };
class CommandSource { virtual bool poll(Command&) = 0; };
```

The existing flatbuffers schema (`telemetry-2026/RemoteControl.fbs` in
Canards-Rocket) already has `ArmFlight`, `CanardsTest`, `CanardsEnable`,
`CanardsDisable`, `StartEstimator` and `Abort`. Propose a mapping to the
ground-station owner (e.g. `StartEstimator` → `EnterPreFlight`). Don't decide
it alone.

## 7. Sim on the board and flight replay (decided: both run the real code on the board)

The user wants:
- **(b)** simulated data streamed over USB into the board;
- **(c)** a flight log replayed from the SD card;

both running the actual flight program, including replaying a flight flown on
old code through new code. A desktop-only harness is not enough by itself.
`env:native` comes almost for free and is still useful for tests.

**What makes it possible:**
1. **Swappable sensor sources:** `SensorSource` has three implementations,
   Live, UsbStream and SdReplay. Everything after it is the same code.
2. **Sensor frames stay in sensor axes, in SI units.** The mount rotation is
   applied in Navigation. Replaying an old log can then pick up a corrected
   mount map.
3. **Log every input, losslessly:**
   - every sensor frame, at full rate;
   - every actuator feedback sample;
   - every ground command, with its timestamp;
   - the git hash and a hash of the config headers, at the start of the log.

   Without this, old flights cannot be replayed. Also log the outputs: state,
   canard status, commands, pulses and safety bits.
4. **Deterministic time:** see section 4.

**Mode selection at boot:** Flight / UsbStream / SdReplay. Bench is a separate
program. Mode rules:
- Flight mode never accepts injected data.
- Stream and replay modes must not count as flight-ready:
  - **USB stream** needs a heartbeat from the host; if it stops, the canards go
    neutral.
  - **Servos may physically move in either mode** (the user wants that), but
    only after an explicit enable command.

**SD replay is open loop.** The recorded motion was caused by the old commands,
so the new code's commands do not change what happens next. Replay answers some
questions but not others:
- It can show whether the state machine changes state at the same moments, and
  whether the controller commands sane values.
- It cannot show whether the new code would have flown straight.

Say this in the tool's output and docs.

**USB stream can run in lockstep with the sim (recommended next step after
replay):**
- Start from the RocketPy sim in launch-controller. It hooks the controller at
  200 Hz in `sim/voyager_sim/flight.py:166-225` and calls the C++ through ctypes
  (`cbridge.py`).
- Replace that ctypes call with a serial round trip:
  1. The sim sends one step of sensor data.
  2. The board runs one cycle and returns its canard commands (or its encoder
     readings, if the servos are live).
  3. The sim advances.
- **Gaps to fill:** that sim reads attitude straight from truth and has only
  gyro noise and bias, with no accelerometer, barometer or magnetometer model.
  The stream needs those added.
- SlidingModeLQRSim already exports CSV telemetry, if a file-based source is
  easier first.

**Bench program** (`bench/`, env `marsv21_bench`): port the useful commands of
Canards-Rocket's `canard_bench/main.cpp`: status, open-loop canard angle, raw
pulse, the `cal N` sweep, choosing which IMU to use, and the ground test. Every
move goes through the Canards subsystem, so the same limits apply.

## 8. What to port, and from where

### 8.1 Attitude controller (LQR)

**Port from Voyager-launch-controller `controller.{h,cpp}` and
`lqr_gain_table.h`, NOT from Canards-Rocket's `src/boilerplate/controller/`.**
The boilerplate copy is out of date in two ways:
- it uses uniform scaling, where launch-controller has **roll-priority
  saturation** (`controller.cpp:116-131`);
- it has the old linkage numbers. Corrected values: push rod **46.05**, shaft
  spacing **45.824**, neutral N2 = **−2.434708°**.

Both copies use a 4° limit and the same gain table.

**The controller:**
- One law per axis: `v = −(k_e·e + k_w·ω + k_s·σ)`. The gains come from a table
  keyed by speed (30–130 m/s, end rows held), with linear interpolation.
- Roll error is clamped to ±10°.
- If any input is not a finite number, all commands are zero.
- Gains are designed for a 5 ms step with a 30 ms servo lag.
- `GNC_GAIN_TABLE_HEADER` lets the sim and tests swap tables.

**Frame conversion: decided to convert, not wrap.** The old controller frame is
x = pitch, y = yaw, z = roll (out of the nose). It is the team standard frame
with its axes relabelled:

| old controller axis | team-standard body axis (new) |
|---|---|
| x (pitch) | y |
| y (yaw) | z |
| z (roll, nose) | x |

So:
- `e_new = (e_old.z, e_old.x, e_old.y)`, and the same for rates.
- Canard i sat at azimuth 90°·i from old +x towards old +y, which is **from body
  +y towards body +z** in the new frame.
- A positive canard angle still rolls the rocket about +nose.

The canard layout measured on the airframe (std frame = new body frame) is:
- CH1 on +y
- CH3 on +z
- CH4 on −y
- CH2 on −z

That gives **canard→channel = {CH1, CH3, CH4, CH2} = {0, 2, 3, 1}**, the same as
Canards-Rocket's `kChannelOfCanard`. **It still assumes CH1 → CH3 is a quarter
turn clockwise seen from the tail. That is unconfirmed; have the user check it.**

**Required test:** feed the same physical situation to the old function (in its
frame) and to the port (in the new frame), and assert identical canard
commands. Do this over a sweep of errors, rates and speeds. Port the existing
host tests too: `test/test_controller.cpp` in launch-controller, which uses
`test/reference_table/`.

**Reference attitude:** "nose to true vertical, keep the roll the rocket has on
entering Control". This was Canards-Rocket's `RollReference::EntryAngle`;
launch-controller's study `sim/studies/canards_state_law.py` models the same
law. Capture it in `Control.init()`, or on the first cycle with valid
navigation.

**Not modelled yet:** the 50 Hz servo update (section 3). Raise it with the
user. The gains may need recomputing in the sim at the real update rate.

### 8.2 Servo control

Canard angle → pulse:
```
pulse = centre + µs_per_deg × (target + PID correction)
```
The PID is an outer loop on the MA4 encoder on the canard shaft.
- **Port from:** Canards-Rocket `src/canards/CanardControl.cpp` (steps 1, 3 and
  4) and `src/canards/pid_tuner/{PidController,ControlMath}.h`. Those files are
  a verbatim copy of pid-tuner `include/`, which keeps bench-tuned gains valid.
- **Behaviour to keep:**
  - Run the position loop at 100 Hz, which is how it was tuned.
  - With no encoder feedback, run open loop from the calibration alone.
  - Use the modelled canard angle (`servoModelStep`) as the LQR's σ input when
    any encoder is unusable.
- **Calibration:** the 2026-11-20 bench values are in Canards-Rocket
  `src/canards/CanardConfig.h`. On every channel a longer pulse turns the canard
  anticlockwise seen from the tail, so µs/deg is negative. The servo PID gains
  are still **zero**. Copy the values only after the canard numbering in 8.1 is
  confirmed.

### 8.3 Hardware wrappers

**`LiveSensorSource`:**
- Wrap the sensor code in the bring-up `main.cpp` (`sensorInit`/`sensorUpdate`).
  The LSM6DSO32 reports mg and mdps; convert to m/s² and rad/s.
- Use the LSM6's data-ready interrupt (PB3) to mark a new sample.
- Use the **LSM6DSO32** (CS PB4). The ASM330 on this board reads WHO_AM_I 0xFF
  and is dead.
- LSM6 mount map in the new frame, measured from gravity readings:
  `body = [[0,0,1],[-1,0,0],[0,-1,0]] · sensor` (determinant +1).
- The ASM330 and magnetometer maps are still placeholders.

**`MarsActuators`:** port Canards-Rocket `src/canards/CanardActuators.{h,cpp}`,
`CanardPins.h` and `pid_tuner/EncoderCapture.h`. Hardware facts, also in the
old handoff:
- Servos on TIM1 at 50 Hz: PE11, PE13, PE14, PA8.
- Servo power on PA4. Whether it is active-high is unverified.
- **The level-shift enable PB12 is active-LOW.** With it released or driven
  high, no pulses reach the connectors.
- Encoder 2 was rewired to PA0.
- **Encoder cables run in reverse order:** servo channel n is read on encoder
  input 3 − n (`kEncoderOfServo = {3,2,1,0}`).

### 8.4 Don't port the Canards-Rocket state machine

It has bugs, and the design above replaces it:
- `setup()` discards the return value of the first state's init
  (`src/main.cpp:359`), so the PRELAUNCH data pointer is null.
- Launch and burnout detection read the split MEKF in `ctx->estimator`, which is
  never initialised or updated. PRELAUNCH also checks accel x, but the nose is z
  in that frame.
- COAST never exits: `airBrakesDone` is never set.
- DROGUE_DESCENT has a comma-operator bug (`... < MAIN_MAX_VEL, data->currentTime)`)
  and leaves at once.
- BOOST always leaves after 2 s, and every timeout constant is 0.

## 9. Placeholders and open questions

**Placeholder rule:** every value without an agreed number is tagged
`PLACEHOLDER` in `include/*.h`, so `grep -n PLACEHOLDER include/` lists what is
left before flight. Never pick a safety-critical number silently. Use a tagged
placeholder and add it to this list.

| Item | Placeholder / status | Who decides |
|---|---|---|
| Boost fallback time ("x seconds") | 2 s (Canards-Rocket's old cut-off, no analysis) | user |
| Control end speed (`kControlEndSpeed`) | 40 m/s; must stay above the ~30 m/s Chute speed | user / GNC |
| Chute speed | ~30 m/s (user said "like 30") | user to confirm |
| Land altitude | 30 m | user |
| Hold times for each transition, state timeouts | guesses | user |
| Launch hold time | 50 ms | user |
| Envelope cut-off: max tilt, max pitch/yaw rate | 30°, 5 rad/s (old placeholders) | GNC |
| Canard limit | 4° (re-check in sim: the sim's roll curve reverses near 4.2°, while team data says 7°) | GNC |
| Control rate and IMU output rate | undecided (section 3) | user |
| RS100 maximum pulse rate | unknown; 50 Hz now | user / datasheet |
| Canard handedness (CH1 → CH3 clockwise from tail) | assumed | user, on the airframe |
| Servo calibration rows, PID gains | bench values exist; gains are zero | bench work |
| Servo-power active level, ASM330 and mag maps, which shaft the encoder sits on | unverified | EE / user |
| Disable-in-flight behaviour (power off vs powered neutral) | proposal: power off | user |
| Latching policy for transient safety faults | proposal: active only for that cycle, sticky for logs | user |
| PreFlight → Init stand-down, re-init on repeated command | proposal | user |
| Command mapping to `RemoteControl.fbs` | proposal | ground-station owner |
| Exact rules and limits for this experimental flight | to come | user |

## 10. Suggested order of work and done-criteria

Each step ends with `pio test -e native` passing, and with both `marsv21` and
`marsv21_bringup` building. Show the user each step before starting the next.

1. **Layout and envs.** Create the folders from section 5 and move the bring-up
   program to `bringup/` with its own env. Add `env:native`. *Done:* all three
   envs build, and the bring-up program behaves as before on hardware (the user
   checks).
2. **`lib/Frames`.** *Done:* tests cover:
   - nose-up gives tilt 0 at the pitch = 90° singularity;
   - q and −q give the same error;
   - the error axes and signs are right;
   - the vertical reference removes tilt and keeps roll;
   - straight-down is handled.
3. **The state machine core:** State, StateMachine, the allowed-transitions
   list, Persistence. *Done:* tests prove the lifecycle order from 6.2 and that a
   refused transition stays put and is counted.
4. **Subsystems:**
   - Navigation, with a fake estimator in tests;
   - Comms types and `CommandList`;
   - Canards, with a fake AttitudeControl;
   - SafetyMonitor.

   *Done:* tests show that each of these ends at neutral:
   - no request;
   - a forbidden request;
   - conflicting requests;
   - each gate;
   - the cut-off latch;
   - lock-out after a launch from Init.

   They also show servo power off in Chute and Land, and enable accepted only on
   the ground.
5. **The seven states and `FlightComputer`.** *Done:* a scripted-flight desktop
   test (fake sensor source, fake estimator) reaches every state in order at the
   expected times. Two more scripted tests:
   - launch while in Init, with canards locked out the whole flight;
   - `NullEstimator`, where every transition falls back to its timeout.
6. **ServoControl** (8.2), with tests against the tuner's behaviour.
7. **The AttitudeControl port** (8.1), with the old-vs-new equivalence test.
8. **The hardware side:**
   - LiveSensorSource and MarsActuators;
   - the SD log sink plus `lib/FlightLog` and `tools/` decoder;
   - SdReplaySource;
   - UsbStreamSource and the sim host in `tools/`.

   *Done:* a recorded pad log replays on the board and reproduces the recorded
   state sequence.
9. **The bench program** (section 7).

**Useful commands:**

```bash
~/.platformio/penv/bin/pio test -e native
```

```bash
~/.platformio/penv/bin/pio run -e marsv21
```

```bash
~/.platformio/penv/bin/pio run -e marsv21 -t upload
```

Monitor (the port name is from the old handoff, so check it):

```bash
~/.platformio/penv/bin/pio device monitor --port /dev/cu.usbmodem3571366932331
```
