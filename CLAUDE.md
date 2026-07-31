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
