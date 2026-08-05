# pylon-track — notes for Claude

Real-time dual-animal (ferret/prey) tracking rig: Basler USB3 camera (pylon SDK) +
OpenCV vision pipeline, an ODrive S1 CAN-bus motor driving a prey chain, and a
LabJack-driven shuttle motor. See `README.md` for the full architecture writeup —
this file is just what a fresh session needs to not re-derive from scratch.

## Build/toolchain — Linux only, and not buildable in most sandboxes

- **Platform: Linux only.** No Windows/macOS support target.
- Requires: Basler pylon SDK (`/opt/pylon` by default), OpenCV with `objdetect`/
  ChArUco support (`libopencv-contrib-dev` on Debian/Ubuntu), LabJack LJM
  (optional — build works without it, LabJack backend just stubs out),
  SocketCAN (`can-utils`, a configured `can0` interface) for the ODrive.
- **If you're running in a sandbox without this toolchain (e.g. a Windows dev
  box), you cannot compile or run this project.** Don't claim to have tested
  or verified behavior beyond static/manual code review unless you've
  confirmed a real build actually happened. Say so explicitly instead of
  guessing.
- `cmake -DPYLON_ROOT=... -DOpenCV_DIR=...` from `build/`; see CMakeLists.txt
  for every override flag. Two real executables come out of one build:
  `ferret_tracker` and `arena_experiment` (see below).

## Two parallel entry points — don't confuse them

- **`src/main.cpp` → `ferret_tracker`**: lighter tool. Camera + `FerretTracker`
  + optional `--experiment <config>` mode (phase tracking, CSV logging via
  `SessionRecorder`/`TrialStateMachine`). **No motor control at all** — no
  PreyMotor, no ChaseController, no ShuttleMotor. Telemetry only.
- **`src/arena_experiment_main.cpp` → `arena_experiment`**: the full rig.
  Subcommands `setup` (one-time interactive hardware calibration —
  `SetupRunner` → arena mask / camera lens / ODrive chain / LabJack shuttle
  calibrators) and `run` (`ExperimentStateManager` — the real trial
  orchestrator: `CameraTrackingService`, `ChaseController` driving `PreyMotor`
  over CAN, `ShuttleMotor` driving LabJack FIO4/FIO5).
- These two binaries independently duplicate a similar "operator reads stdin
  for s/e/r keys" pattern and both had the same Ctrl-C shutdown hang bug —
  when fixing one, check the other.

## Motor/LabJack subsystem (this is where most bugs cluster)

- `PreyMotor` (chain motor) wraps `ODriveCan` (raw SocketCAN CANSimple
  client). `ChaseController` runs its own ~20ms thread calling
  `PreyMotor::apply()`; `ExperimentStateManager::chase_feed_loop()` runs a
  separate ~5ms thread that also reads `PreyMotor` status
  (`read_position_turns()`, to feed `ShuttleMotor`). Both threads hit the same
  `ODriveCan` socket — it now has its own mutex (`io_mutex_`) after a data-race
  fix; if you add another concurrent caller, make sure it still goes through
  the locked public API, not `send_frame`/`recv_frame` directly.
- `ODriveCan::recv_frame()` demuxes the shared CAN RX queue into a per-cmd_id
  cache (`rx_cache_`) instead of discarding frames that don't match what the
  calling method asked for. This matters for any new Get_* telemetry getter:
  without going through this cache, a fast poller (e.g.
  `get_encoder_estimates`, called every sample tick) will silently vacuum up
  and discard frames a different, slower poller (e.g. `get_iq`,
  `get_active_errors`) is waiting for — a real bug that made Get_Iq/Get_Error
  always time out regardless of RTR requests, fixed on `inertia-calibration`.
  Route any new Get_* getter through `recv_frame()`; don't `read()` the
  socket directly.
- This lab's ODrive reports `Iq_Measured` with the opposite sign convention
  from the encoder's positive-velocity direction (current-sensor/phase
  polarity — confirmed on hardware by checking `iq_measured_a` against
  `measured_turns_s` during a clean spin-up, not a guess). Iq isn't read in
  any production control path today, only in
  `tests/motor_inertia_calibration_test.cpp`'s dynamic-fit regression, where
  it's corrected via `--iq-sign` (defaults to `-1` for this rig — if it's
  ever rewired/reconfigured, the tell is `chain_inertia_kg_m2`/
  `chain_viscous_friction_nm_s_per_rad`/`chain_static_friction_nm` all
  fitting negative together, which is physically impossible). If Iq is ever
  read from a production path, remember the sign is inverted on this rig.
- This rig's ODrive velocity loop (`axis0.controller.config.vel_gain` /
  `vel_integrator_gain`, set via `odrivetool`, not this repo) needed
  retuning after raising the CAN current limit from 40A to 60A
  (`kDefaultCurrentLimitA` in `odrive_can.cpp`) — the higher ceiling
  unmasked an underdamped loop the old 40A limit had been saturating
  current before it could visibly oscillate. Original gains (~0.167 /
  ~0.333, a 2:1 integrator:proportional ratio) produced a sustained
  limit-cycle around the target speed that didn't damp out no matter how
  long you waited; lowering `vel_integrator_gain` helped but traded off
  against low-speed tracking — too low and the real chain's load produces a
  persistent (not oscillating) steady-state undershoot, since a weaker
  integrator can't build enough torque fast enough to close a bigger load's
  error. A single fixed gain pair may not serve the whole practical speed
  range on this rig; `ODriveCan::set_vel_gains()` /
  `PreyMotor::set_vel_gains()` (`Set_Vel_Gains`, CAN cmd `0x01b`, runtime-
  only — not persisted, `odrivetool`'s `save_configuration()` is separate)
  exist for exactly this, and
  `tests/motor_inertia_calibration_test.cpp --schedule-gains` linearly
  interpolates vel_integrator_gain between --vel-integrator-min/-max by
  target speed (fixed --vel-gain) *between calibration steps* — not wired
  into any live production control path, since switching gains mid-motion
  needs bumpless-transfer handling this repo doesn't have yet.
  If chain motor speed steps oscillate again after any future current-limit
  or gain change, this is the same class of issue, not a code bug.
- `PreyMotor`'s breakaway kick (`kKick*` in `lab_motion_limits.h`) commands a
  flat, fixed speed when starting a low-speed move from rest, then swaps to
  the literal target once *measured* velocity (not a timer) reaches a
  fraction of target — feedback-driven because how long breakaway actually
  takes depends on real load, not a guess. `compute_torque_ff_nm()`
  deliberately takes the literal target, never the kick-adjusted value — an
  earlier version fed the boosted value in, so every kick-end handoff looked
  like a huge fictitious deceleration and injected a large spurious braking
  torque right as the kick relaxed. If you touch either of these two
  functions, keep that decoupling — recombining them reintroduces that bug.
- `ShuttleMotor` (LabJack, `include/motor/shuttle_motor.h`) replaced an older
  `TrapDoorMotor`/`LabJackDAC` design (deleted). It drives two LabJack
  **analog** FIO pins (FIO4/FIO5 — not fixed digital, so "high" is a real
  voltage, `high_voltage` config field, default 5.0V) as a crude H-bridge:
  idles in a small wobble, fires a directional pulse when the chain's tracked
  position crosses configured hallway-end thresholds.
- `arena_experiment.json`'s `shuttle` section replaced the old `trap_door`
  section. If you see "trap door" or `TrapDoorMotor`/`LabJackDAC`/`LabJackIo`
  referenced anywhere, that's stale — it was fully replaced.

## Branch history quirk worth knowing

- `main` underwent a large architecture rewrite (motor CAN rewrite, vision
  pipeline extraction, experiment state manager, ChArUco setup tooling)
  while this repo's feature branches were mid-flight. `web-ui-odrive` was
  rebuilt from scratch on top of the current `main` rather than merged —
  if you see old commit messages about a Flask web UI or a simple serial
  ODrive class, that work was intentionally superseded, not lost.

## Known-fragile spots (as of the last full review pass)

A full bug-hunt pass found ~21 issues (calib.npz save corruption, CAN thread
races, Ctrl-C hangs, Kalman filter not coasting through missed detections,
etc.) — most have been fixed. If something in motor/vision/calibrate looks
subtly wrong, check whether it's already a known finding before re-deriving
the whole analysis; grep recent commit messages for context first.
