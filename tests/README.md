# Camera Calibration Test Suite

Hardware-in-the-loop tools to find the best `camera_config.json` values for the
Basler a2A1920-160umPRO arena camera. These are **not unit tests** — every tool
needs the camera attached and runs on the lab Linux box.

Built automatically with the main project (disable with
`cmake -DBUILD_CALIBRATION_TESTS=OFF ..`). Binaries land in `build/bin/`.
All outputs go to `tests/output/<suite>/<timestamp>_<label>/` (gitignored).

| Tool | Purpose |
|------|---------|
| `test_one_time_setup` | Apply fixed rig settings once; verification frame + `setup_report.json` |
| `test_param_sweep` | Parameter sweeps + resolution / binning / compound camera presets |
| `test_latency` | Two-object tracking benchmark: speeds, centroids, distance, latency |
| `test_mount_height` | Per-height resolution check: annotated stills + the latency benchmark |
| `test_motor_inertia_calibration` | Motor-only (no camera): calibrates chain inertia/friction from a step-response sweep |

---

## Quick start (lab machine)

### 1. Prerequisites

- Basler **a2A1920-160umPRO** on USB3, arena lit, camera mounted
- Linux with pylon SDK (`/opt/pylon` or set `-DPYLON_ROOT=` at cmake time)
- Repo on branch `feature/calibration-tests` (or merged main once PR lands)

```bash
# From repo root — first time or after pulling new commits
git fetch origin
git checkout feature/calibration-tests
git pull origin feature/calibration-tests

mkdir -p build && cd build
cmake ..
make ferret_tracker test_one_time_setup test_param_sweep test_latency test_mount_height

# USB permissions (once per machine)
sudo make install_udev
# unplug/replug camera, or: sudo udevadm control --reload-rules && sudo udevadm trigger
```

Verify the camera is visible:

```bash
lsusb | grep -i basler
# optional: /opt/pylon/bin/pylonviewer
```

### 2. Working directory

All commands below assume **`cd build`** — paths to specs use `../tests/...`.

```bash
cd build
export PYLON_CAMERA_CONFIG=../src/camera/camera_config.json   # optional; default is build/bin/camera_config.json
```

After `make`, `camera_config.json` is copied to `build/bin/`. Edit either that
file or `src/camera/camera_config.json` (re-copy or symlink if you prefer the src copy).

### 3. Recommended full calibration run

Run in this order. Keep arena lighting **stable** across sweeps. Let the
camera **warm up** 5–10 min before sweeps.

```bash
cd build

# Step 0 — one-time rig check (after mount / lens / lighting change)
./bin/test_one_time_setup --settings ../tests/one_time_settings.json

# Step 1 — exposure (pick row from sweep.csv: high laplacian_var, mean_gray ~128–180)
./bin/test_param_sweep --sweep ../tests/sweep_configs/exposure_sweep.json
# optional wider range:
./bin/test_param_sweep --sweep ../tests/sweep_configs/exposure_extended_sweep.json

# Step 2 — gain (quantify noise cost; keep as low as brightness allows)
./bin/test_param_sweep --sweep ../tests/sweep_configs/gain_sweep.json

# Step 3 — AOI / resolution (16 presets; check fov_mm + achieved_fps)
./bin/test_param_sweep --sweep ../tests/sweep_configs/resolution_sweep.json
# if mount height ≠ 1.2 m:
./bin/test_param_sweep --sweep ../tests/sweep_configs/resolution_sweep.json --gsd 1.29

# Step 4 — fine-tune vertical centering (optional)
./bin/test_param_sweep --sweep ../tests/sweep_configs/offset_y_sweep.json

# Step 5 — lock winners into camera_config.json, then validate height
./bin/test_mount_height --height-cm 120 --duration 30 --warmup-secs 30

# Step 6 — latency + distance accuracy (empty arena during warmup!)
./bin/test_latency --duration 30 --warmup-secs 30

# Step 7 — production smoke test
./bin/ferret_tracker --display
```

### 4. Optional / secondary sweeps

Run only if you are exploring a specific trade-off:

```bash
cd build

./bin/test_param_sweep --sweep ../tests/sweep_configs/frame_rate_sweep.json
./bin/test_param_sweep --sweep ../tests/sweep_configs/frame_rate_enable_sweep.json
./bin/test_param_sweep --sweep ../tests/sweep_configs/black_level_sweep.json
./bin/test_param_sweep --sweep ../tests/sweep_configs/gamma_sweep.json
./bin/test_param_sweep --sweep ../tests/sweep_configs/offset_x_sweep.json
./bin/test_param_sweep --sweep ../tests/sweep_configs/scaling_sweep.json
./bin/test_param_sweep --sweep ../tests/sweep_configs/binning_sweep.json
./bin/test_param_sweep --sweep ../tests/sweep_configs/ultra_short_exposure_sweep.json
./bin/test_param_sweep --sweep ../tests/sweep_configs/throughput_sweep.json
```

### 5. Where outputs go

```text
tests/output/
  one_time_setup/<timestamp>_rig/
    setup_report.json
    <timestamp>_one_time_verify_f000.png
  param_sweep/<timestamp>_<label>/
    sweep.csv | resolution.csv | binning.csv | camera_preset.csv
    <timestamp>_<prefix>_fNNN.png
  latency/<timestamp>_latency/
    frames.csv
    summary.csv
  mount_height/<timestamp>_h120cm/
    stills/          # annotated PNGs ~1 Hz
    frames.csv
    summary.csv
```

Inspect CSVs with any spreadsheet tool, or:

```bash
column -t -s, tests/output/param_sweep/*/sweep.csv | less -S
```

### 6. CLI reference (all tools)

| Flag | Tools | Description |
|------|-------|-------------|
| `--settings <json>` | `test_one_time_setup` | Rig settings file (default `tests/one_time_settings.json`) |
| `--sweep <json>` | `test_param_sweep` | **Required.** Sweep spec under `tests/sweep_configs/` |
| `--height-cm <cm>` | `test_mount_height` | **Required.** Physical mount height |
| `--duration <s>` | `test_latency`, `test_mount_height` | Capture length (default 30) |
| `--warmup-secs <s>` | `test_latency`, `test_mount_height` | MOG2 background warmup (default 30) |
| `--gsd <mm/px>` | `test_param_sweep`, `test_latency`, `test_mount_height` | Override GSD (default 1.035 @ 1.2 m) |
| `--camera-config <path>` | all | Override `camera_config.json` lookup |
| `--output <dir>` | all | Output root (default `tests/output`) |
| `--verbose` | all | Debug logging |

**Usage one-liners:**

```bash
test_one_time_setup  --settings <json> [--camera-config <path>] [--output <dir>] [--verbose]
test_param_sweep     --sweep <json> [--gsd <mm/px>] [--camera-config <path>] [--output <dir>] [--verbose]
test_latency         [--duration <s>] [--warmup-secs <s>] [--gsd <mm/px>] [--camera-config <path>] [--output <dir>] [--verbose]
test_mount_height    --height-cm <cm> [--duration <s>] [--warmup-secs <s>] [--gsd <mm/px>] [--camera-config <path>] [--output <dir>] [--verbose]
```

### 7. After calibration — production tracker

Copy winning values into `src/camera/camera_config.json` (or `build/bin/camera_config.json`),
then:

```bash
cd build
./bin/ferret_tracker              # headless telemetry on stdout
./bin/ferret_tracker --display    # live overlay (needs DISPLAY / desktop session)
./bin/ferret_tracker --verbose --log-file /tmp/ferret.log
./bin/ferret_tracker --camera-config /path/to/camera_config.json
```

Keep the arena **empty for 30 s** on startup while the background model warms up.

---

## `test_one_time_setup` — one-time rig settings

Run **once** after mounting the camera, changing the lens, or relighting the arena.
Applies the fixed `camera` block from `one_time_settings.json`, grabs a short
verification capture, read backs GenICam values, and writes `setup_report.json`.

Flat-field / vignetting correction is still manual in pylon Viewer — see
`manual_steps` in the JSON.

```bash
./bin/test_one_time_setup --settings ../tests/one_time_settings.json
```

Outputs: `setup_report.json`, one verification PNG, pass/fail on mean gray,
clipping, and fps vs target.

After a pass, copy the `applied_settings` block into `src/camera/camera_config.json`.

Settings file: [`one_time_settings.json`](one_time_settings.json) — edit the
`camera` block and `verification` thresholds before running.

---

## `camera_config.json` fields (production + sweeps)

[`src/camera/camera_config.json`](../src/camera/camera_config.json) is the baseline for all
sweeps. Newer fields are optional in older copies (defaults apply).

| Field | Default | Notes |
|-------|---------|-------|
| `exposure_time_us` | `2000` | µs |
| `exposure_time_mode` | `Standard` | `Standard` or `UltraShort`; alias `Common` → `Standard` |
| `gain_db` | `6.0` | dB, 0–24 user-facing |
| `width` / `height` | `1920` / `960` | AOI |
| `offset_x` / `offset_y` | `0` / `120` | AOI position |
| `frame_rate_enable` | `true` | `false` = max unconstrained fps |
| `frame_rate_fps` | `200.0` | Target when enable is true |
| `black_level` | `0` | ≤ 64 recommended |
| `gamma` | `1.0` | Keep at 1.0 for tracking |
| `binning_horizontal` / `binning_vertical` | `1` | Mutually exclusive with scaling |
| `binning_selector` | `Sensor` | `Sensor` or `FPGA` (Pylon enum `Region1`) |
| `scaling_horizontal` | `1.0` | &lt; 1.0 in-camera downscale |
| `reverse_x` / `reverse_y` | `false` | Mount orientation |
| `device_link_throughput_limit` | `Off` | `On` + `device_link_throughput_mbps` |

---

## `test_param_sweep` — parameter and preset sweeps

One binary auto-detects the spec format:

| Spec shape | Mode | Output CSV |
|------------|------|------------|
| `"parameter"` + `"values"` | Single-parameter sweep | `sweep.csv` |
| `"presets"` (default) | AOI width×height resolution | `resolution.csv` |
| `"presets"` + `"preset_type": "binning"` | Binning combinations | `binning.csv` |
| `"presets"` + `"preset_type": "camera"` | Compound presets (exposure mode, throughput, …) | `camera_preset.csv` |

### Available sweep configs (`tests/sweep_configs/`)

| File | What it sweeps |
|------|----------------|
| `exposure_sweep.json` | Exposure (250–4000 µs) |
| `exposure_extended_sweep.json` | Exposure 19 µs–5000 µs (Standard mode) |
| `ultra_short_exposure_sweep.json` | Standard + UltraShort exposure presets |
| `gain_sweep.json` | Gain 0–24 dB |
| `frame_rate_sweep.json` | Target fps cap |
| `frame_rate_enable_sweep.json` | Capped vs free-run max fps |
| `resolution_sweep.json` | 16 AOI width×height×offset combos |
| `offset_x_sweep.json` / `offset_y_sweep.json` | AOI centering |
| `black_level_sweep.json` | Black level 0–64 |
| `gamma_sweep.json` | Gamma 0.5–2.0 |
| `scaling_sweep.json` | In-camera scaling (requires binning off) |
| `binning_sweep.json` | Sensor vs FPGA (`Region1`) binning 1×1–4×4 |
| `throughput_sweep.json` | USB throughput limit on/off |

### Single-parameter mode

Holds every setting at the `camera_config.json` baseline and steps **one**
parameter through the values in a sweep spec:

```bash
./bin/test_param_sweep --sweep ../tests/sweep_configs/exposure_sweep.json
```

Recommended order: **one_time_setup** → **exposure** → **gain** → **resolution**
→ **offset_y** (fine-tune) → **mount_height** → **latency**.

Spec format (single-parameter):

```json
{
	"parameter": "exposure_time_us",
	"values": [250, 500, 1000, 2000, 4000],
	"frames_per_value": 50,
	"save_images": 3
}
```

Outputs per run:

- `sweep.csv` — one row per value: achieved fps, mean gray, stddev (contrast),
  % clipped pixels low/high, Laplacian variance (sharpness/noise).
- Sample PNGs named `<timestamp>_<parameter>_<value>_f<NNN>.png`.

How to pick a winner (see Basler notes below):

- `mean_gray` in ~128–180 with `clipped_*_pct` near zero.
- Highest `laplacian_var` at acceptable brightness usually wins for exposure
  (less motion blur); for gain, rising `laplacian_var` with rising `stddev`
  at the same scene means you are amplifying noise, not signal.
- `achieved_fps` confirms the value doesn't choke the frame rate
  (exposure must fit the frame period: 5000 µs at 200 fps).

### Resolution / AOI preset mode

Steps through **width × height** (and optional `offset_x` / `offset_y`) presets.
Cropping changes **field of view in mm** and achievable **fps**, not mm/px per
pixel (GSD is set by lens + mount height). Use this to pick an AOI that covers
enough arena while sustaining the target frame rate.

```bash
./bin/test_param_sweep --sweep ../tests/sweep_configs/resolution_sweep.json
```

Spec format (`resolution_sweep.json`):

```json
{
	"gsd_mm_px": 1.035,
	"frames_per_preset": 50,
	"save_images": 2,
	"presets": [
		{
			"label": "production_1920x960_y120",
			"width": 1920,
			"height": 960,
			"offset_x": 0,
			"offset_y": 120
		}
	]
}
```

Override GSD from the CLI if your mount height differs from 1.2 m:
`--gsd 1.29` (e.g. 1.5 m mount).

Outputs per run:

- `resolution.csv` — per preset: label, width, height, offsets, total_px,
  `fov_width_mm`, `fov_height_mm`, megapixels, achieved fps, image metrics.
- Sample PNGs — `<timestamp>_resolution_<label>_fNNN.png`.

How to pick a winner:

- `fov_width_mm` × `fov_height_mm` must cover the full arena footprint.
- `achieved_fps` should meet or exceed your tracking target (e.g. 200).
- Compare sample images at the same lighting — tighter crops trade coverage
  for fps; use `test_mount_height` afterward to confirm blobs stay ≥200 px².

### Binning preset mode

```bash
./bin/test_param_sweep --sweep ../tests/sweep_configs/binning_sweep.json
```

```json
{
	"preset_type": "binning",
	"presets": [
		{
			"label": "sensor_2x2",
			"binning_selector": "Sensor",
			"binning_horizontal": 2,
			"binning_vertical": 2
		},
		{
			"label": "fpga_2x2",
			"binning_selector": "FPGA",
			"binning_horizontal": 2,
			"binning_vertical": 2
		}
	]
}
```

`binning_selector: "FPGA"` maps to Pylon `Region1` on ace 2 USB. 2×2 binning
doubles effective pixel size — rescale GSD and re-run
`test_mount_height` before trusting mm measurements.

### Compound camera preset mode

For settings that must change together (exposure mode + exposure time,
throughput cap + Mbps):

```bash
./bin/test_param_sweep --sweep ../tests/sweep_configs/ultra_short_exposure_sweep.json
```

```json
{
	"preset_type": "camera",
	"presets": [
		{
			"label": "standard_500us",
			"exposure_time_mode": "Standard",
			"exposure_time_us": 500
		},
		{
			"label": "ultra_5us",
			"exposure_time_mode": "UltraShort",
			"exposure_time_us": 5
		}
	]
}
```

## `test_latency` — two-object latency benchmark

Runs the **production pipeline** (MOG2 background subtraction + Kalman,
`FerretTracker`) and times grab → distance-between-objects per frame.

```bash
./bin/test_latency --duration 30 --warmup-secs 30
```

Protocol:

1. Keep the arena **empty** during the warmup window (background learning).
2. Introduce the two moving objects when warmup ends; vary their speeds
   between runs (slow / medium / fast) and compare CSVs.

Outputs per run:

- `frames.csv` — per frame: `frame_index, camera_ts_ticks, host_time_us,
  speed1_mm_s, speed2_mm_s, c1/c2 centroids (px and mm), distance_mm,
  latency_us, valid1, valid2`.
- `summary.csv` — frames, achieved fps, valid-pair %, latency
  mean/p50/p95/max.

Latency here is host-side processing latency (grab callback → distance
computed). USB transfer/exposure time is not included; add ~1 frame period
for sensor-to-decision budgeting.

## `test_mount_height` — mounting height validation

Height can't be swept automatically — mount the camera at a candidate height,
run once per height:

```bash
./bin/test_mount_height --height-cm 120 --duration 30
```

What it does:

- Rescales GSD linearly from the 1.2 m baseline (`1.035 mm/px`) so mm
  measurements stay correct at the entered height.
- Saves annotated stills (~1 Hz) into `stills/`: contours, bounding boxes,
  and blob areas in px². The production tracker rejects blobs **under
  200 px²** — every object must stay comfortably above that at the chosen
  height.
- Runs the full suite-2 measurement loop; `frames.csv` gains a `height_cm`
  column so runs at different heights concatenate cleanly.

Accuracy check per height: place the two objects at a tape-measured
separation, read `distance_mm` from the CSV, compare. Repeat with the objects
moving to confirm tracking holds.

Mind that raising the camera trades resolution for coverage:

| Height | GSD (mm/px) | 200 px² blob is ~ |
|--------|-------------|--------------------|
| 1.0 m | 0.86 | 12×12 mm object |
| 1.2 m | 1.035 | 15×15 mm object |
| 1.5 m | 1.29 | 18×18 mm object |
| 2.0 m | 1.73 | 24×24 mm object |

---

## `test_motor_inertia_calibration` — chain motor inertia/friction calibration

Motor-only (no camera, Pylon, or LabJack needed) — same linkage as
`test_distance_moving`/`test_hunt_sim`. Requires the ODrive CAN chain motor
connected and `can0` up. **The chain must be a closed loop (no physical
end)** — this test spins continuously for several minutes and does not track
position, unlike `test_odrive_move`/`test_hunt_sim` which bound every move by
distance.

### What it does

Commands a sweep of step velocity changes and times how long the motor takes
to reach each one, then fits a physical model relating motor torque to
acceleration, speed, and static friction. The result is `chain_inertia_kg_m2`,
`chain_viscous_friction_nm_s_per_rad`, and `chain_static_friction_nm` — three
numbers that let `PreyMotor` push a feed-forward torque (`Set_Input_Vel`'s
`torque_ff`, previously always 0) alongside every velocity command, so the
ODrive's own current-limited PI loop has less error to close and spins up
faster. Feed-forward is a no-op ( `torque_ff = 0`) until these values are
non-zero in `config/arena_experiment.json`'s `motor` section — running this
test does not change motor behavior unless you save its output there
(`--write-config`) or paste it in by hand.

### How it works — kick smoke test, then two trials, then two regressions

0. **Kick smoke test (before Trial A):** `PreyMotor` commands a flat, fixed
   `--kick-speed` (default 5.0 turns/s) when starting a low-speed move from
   rest (a breakaway kick — see `kKick*` in
   `include/motor/lab_motion_limits.h`), swapping to the literal target once
   measured velocity reaches `--kick-cutoff-frac` (default 80%) of it — both
   trials below command through that same path. Before Trial A runs, this
   test tries one breakaway from a stop to `--rps-min` (the hardest target in
   the sweep) and reports whether it actually settled, so a kick that doesn't
   work on this hardware is caught in seconds instead of after minutes of
   sweeping. `--no-kick-smoke-test` skips this.
1. **Trial A (cumulative ramp):** starting from a stop, step to `--rps-min`
   (default 1.0 turns/s), time how long it takes to settle, hold there for
   `--hold-s` (default 2s), then step to the *next* target from wherever the
   motor already is (e.g. 1.0 → 1.2 → 1.4 → ...), repeating up to `--rps-max`
   (default 6.0) in `--rps-step` increments (default 0.2). Then decelerate
   from the top target back to 0 (also timed) and dwell `--hold-s` at 0.
2. **Trial B (reset every step):** for every target in the same list, spin up
   from a full stop (timed), hold, spin back down to 0 (timed), hold, then
   move to the next target — always starting from 0. This isolates each
   step's response instead of chaining them, and (unlike Trial A) times a
   deceleration for every single step, not just once at the end.
3. Both trials command the target continuously (feeding the ODrive watchdog)
   and sample velocity + motor current (`Get_Iq`) at `--sample-hz` (default
   100 Hz) throughout — during the ramp *and* the post-settle dwell — logging
   every sample, not just per-step timestamps.
4. **Timing cross-check regression:** `settle_time_s ≈ a + b·|Δturns/s|`, a
   simple least-squares fit across every step from both trials. Printed for
   sanity-checking the dynamic fit below, not used for the saved calibration.
5. **Dynamic regression (the actual calibration):** every consecutive sample
   pair within a step gives one data point — measured torque
   (`Iq_measured × --torque-constant`) against the discrete angular
   acceleration and velocity between those two samples. Least squares across
   every such point from both trials (not just step endpoints) fits
   `torque = J·α + B·ω + τ_c·sign(ω)` for inertia `J`, viscous friction `B`,
   and static/Coulomb friction `τ_c`. Reports R² alongside the fit.

"Reached" a target means velocity stayed within a band continuously for
`--settle-hold-s` (default 0.2 s) — a single in-band sample doesn't count, to
reject overshoot bouncing through the band. The band is `--settle-tol-pct`
(default 2.5%) of the target speed, floored at 0.05 turns/s — never tighter
than that floor (so low-speed steps aren't held to a stricter band than
before), only loosening once 2.5% of the target exceeds it (above ~2 rps at
the default). The floor also covers the descent-to-0 step after every ramp,
where a pure percentage would be an unsatisfiable 0-width band.

### If a step won't settle at a specific speed

A single target failing to settle **does not** abort the whole sweep — it's
logged, recorded in `steps.csv` with `ok=0`, and the sweep moves on to the
next target. Only Ctrl-C, or `test_motor_inertia_calibration` failing to
settle on 5 targets *in a row* (which looks systemic — e.g. a dropped CAN
connection — rather than one bad speed), stops the run early.

Before giving up, if the step is still unsettled right at `--max-step-wait-s`
but the last measured velocity is already within `--near-tol` (default 0.1)
of target, it gets **one** `--grace-s` (default 1 s) extension instead of an
immediate failure — a slow final approach into tolerance is still real
dynamics worth letting finish, not a sign of something wrong, and either way
`settle_time_s` in `steps.csv` reflects the true total time it took (and
`used_grace` flags that it needed the extra second). Only steps that are
*still* not near target when the grace check happens skip straight to
failing; only one extension is granted per step, so a step that's still
oscillating or genuinely stuck can't stall the sweep indefinitely.

The timeout log line includes the ODrive's `active_errors`/`disarm_reason`
and `axis_state` at that moment, the measured velocity **range** and last
value seen during the attempt, the peak `|Iq|`, and whether the grace
extension was already used and still didn't help — plus a classification:

- **"steady above/below target by X turns/s"** — velocity wasn't bouncing
  (swing ≤ 2× the settle band), it's parked a fixed amount off — including
  overshot-and-stuck-high, not just undershoot.
- **"oscillating through the target (hunting)"** — velocity swung above and
  below the target repeatedly without holding inside the settle band. This is
  a velocity-loop stability symptom, not a "couldn't get there" symptom.
- **"swinging without settling"** — moved a lot but stayed on one side of
  the target (didn't cross it) — e.g. still mid-ramp or overshot once and
  hasn't come back down yet.

If speeds above some threshold consistently show **"oscillating through the
target"** with a swing on the order of ±0.1 turns/s or more (not just a
narrow resonance pocket — the same pattern at every speed above the
threshold), that's a velocity-loop tuning symptom, and it may be related to
`kDefaultCurrentLimitA` in `src/motor/odrive_can.cpp` (currently 60 A): a
lower current limit can *mask* an underdamped velocity loop by saturating
current before it can overshoot, so raising the limit can turn a slow-but-
smooth response into a faster-but-oscillating one. To tell them apart:

1. Check `max |Iq|` in the timeout log / `iq_measured_a` in `samples.csv` for
   the failing speeds. Pinned near the current limit the whole time → torque/
   back-EMF saturation (a real ceiling for this speed, not a tuning issue).
   Swinging up and down with velocity rather than pinned → the loop actively
   fighting an oscillation (a tuning issue).
2. As a diagnostic (not a permanent fix), try temporarily lowering
   `kDefaultCurrentLimitA` back toward 40 in `src/motor/odrive_can.cpp` and
   rebuilding. If the oscillation above the threshold goes away, the real fix
   is retuning the ODrive's velocity-loop gains for the higher current
   ceiling (`odrivetool`: `axis0.controller.config.vel_gain` /
   `vel_integrator_gain`), not this test's code.
3. If you just need usable calibration data now while investigating further,
   run with `--rps-max` set just below wherever it starts failing (e.g.
   `--rps-max 4.4`) — the regression is valid over whatever range you
   actually swept, it doesn't need to cover 1.0–6.0.

Loosening `--settle-tol-pct` or `--settle-hold-s` will make either symptom
disappear from the log without fixing the underlying limit — treat that as
confirmation of a real issue, not a fix for one.

### Steps to run it

```bash
cd build
./bin/test_motor_inertia_calibration --config ../config/arena_experiment.json
```

Add `--write-config` to save the fitted values directly into the config's
`motor` section (merge-write — other sections are untouched) instead of
copy-pasting the printed block yourself:

```bash
./bin/test_motor_inertia_calibration --config ../config/arena_experiment.json --write-config
```

Ctrl-C aborts and stops the motor at any point; partial samples/steps are
still written to CSV but no regression is run on an aborted sweep.

### Variables (CLI flags)

| Flag | Default | Meaning |
|------|---------|---------|
| `--config <path>` | resolved beside binary / `config/` | `arena_experiment.json` (needs `motor.chain_mm_per_motor_turn` or `pulley_radius_m`, and `motor.can_interface`) |
| `--rps-min <turns/s>` | `1.0` | First (lowest) sweep target |
| `--rps-max <turns/s>` | `6.0` | Last (highest) sweep target |
| `--rps-step <turns/s>` | `0.2` | Increment between targets |
| `--hold-s <s>` | `2.0` | Dwell time after settling at each target (and at 0) |
| `--settle-tol-pct <%>` | `2.5` | ± band around the target that counts as "reached", as a percent of the target speed, floored at 0.05 turns/s |
| `--settle-hold-s <s>` | `0.2` | Time the velocity must stay inside the band before it's called settled |
| `--sample-hz <Hz>` | `100` | Velocity/Iq sampling (and command re-send) rate during ramps and dwells |
| `--torque-constant <N·m/A>` | `0.0827` | ODrive motor torque constant (`odrivetool`: `axis0.motor.config.torque_constant`) used to convert `Iq_measured` into torque |
| `--max-step-wait-s <s>` | `10` | Give up on a single step after this long — unless the grace extension below applies (stall/fault guard) |
| `--near-tol <turns/s>` | `0.1` | If still unsettled right at `--max-step-wait-s` but within this of target, grant one `--grace-s` extension instead of giving up |
| `--grace-s <s>` | `1.0` | Length of that one-time extension. `steps.csv`'s `used_grace` column and `settle_time_s` reflect whichever attempt it took |
| `--iq-sign <1 or -1>` | `-1` | Multiplies `Iq_Measured` before the dynamic fit. Defaults to `-1` because this rig's `Iq_Measured` sign convention disagrees with the encoder's positive-velocity direction (confirmed on hardware). Pass `1` only if that's ever fixed/rewired — if `chain_inertia_kg_m2`, `chain_viscous_friction_nm_s_per_rad`, and `chain_static_friction_nm` all come out negative together, that's this sign issue again, not noise |
| `--kick-speed <turns/s>` | `LabMotionLimits::kKickFixedTurnsS` (5.0) | Flat breakaway-kick speed, regardless of target |
| `--kick-cutoff-frac <0-1>` | `LabMotionLimits::kKickCutoffFraction` (0.8) | Swap from the kick to the literal target once \|measured\| reaches this fraction of \|target\| |
| `--no-kick-smoke-test` | off | Skip the pre-Trial-A kick smoke test |
| `--output <dir>` | `tests/output` | Root for `motor_inertia_calibration/<timestamp>/{samples,steps}.csv` |
| `--write-config` | off | Merge-write the fitted values into `--config`'s `motor` section |
| `--verbose` | off | Debug logging |

### Output

- `samples.csv` — every logged sample: `t_s, trial, step_index, direction,
  target_turns_s, measured_turns_s, iq_measured_a, iq_valid`.
- `steps.csv` — one row per up/down transition: `trial, step_index,
  direction, from_turns_s, to_turns_s, delta_turns_s, settle_time_s,
  end_measured_turns_s, max_vel_turns_s, used_grace, ok`. Kick-tuning
  attempts (trial `0`) aren't included — they're a separate pre-sweep pass,
  not part of the calibration.
- Printed summary: the timing cross-check fit, then the dynamic fit's R² and
  the three calibrated values, formatted ready to paste into
  `config/arena_experiment.json`'s `motor` section.

`Get_Iq`/`Get_Error` are requested on demand over CAN (an RTR frame) rather
than relying on the ODrive's cyclic-broadcast config for those two messages
— unlike `Get_Encoder_Estimates`/`Heartbeat`, they aren't cyclic by default,
and this driver previously had no way to ask for them, so `max |Iq|` in the
timeout log and `iq_measured_a` in `samples.csv` always read `0`/invalid no
matter how long it waited. If you're on an older build without this fix and
still see `max |Iq| = 0.000000 A` on every single step (not just the failing
ones), that's the symptom — check `axis_error`/`axis_state` instead in the
meantime, since those come from Heartbeat and are unaffected.

If fewer than half the `Get_Iq` reads still succeed after this fix, the tool
warns that something is still off (e.g. the ODrive genuinely isn't answering
RTR requests on this firmware version) and the dynamic fit (not the timing
cross-check) should be treated as unreliable. A fitted `chain_inertia_kg_m2
<= 0` is flagged the same way — physically impossible, so the run's numbers
shouldn't be trusted as-is.

## Basler calibration notes

Distilled from the [Basler image-quality docs](https://docs.baslerweb.com/optimizing-image-quality)
for this rig (Mono8, 200 fps, moving targets):

**Motion blur budget.** Blur in pixels = `speed_mm_s × exposure_s / GSD`.
Keep it under ~1 px or contours smear and centroids lag. At 1 m/s and
1.035 mm/px that means **exposure < ~1 ms**; a sprinting ferret (~2 m/s)
wants < 500 µs. This bounds the useful exposure sweep range — compensate
lost light with illumination, not gain.

**Brightness target.** Aim for a mean gray of ~50–70 % of range with no
clipping at either end. Clipped regions carry no gradient → background
subtraction and contours fail there. Black level: Basler recommends ≤ 64.

**Gain is a last resort.** Gain amplifies signal and noise equally — SNR does
not improve. Raise illumination or exposure (within the blur budget) first;
use the gain sweep to quantify the noise cost (`laplacian_var` and `stddev`
rise together on a static scene = noise).

**Exposure vs frame rate.** Exposure must fit in the frame period
(at 200 fps: ≤ 5000 µs minus readout). If `achieved_fps` drops during an
exposure sweep, the value is throttling acquisition.

**Useful pylon Viewer features** (one-time, before locking values into
`camera_config.json`):

- *Automatic Image Adjustment* — quick baseline for exposure/gain under the
  actual arena lighting; read back the chosen values and use them to center
  the sweep ranges.
- *Flat-Field Correction wizard* — corrects vignetting/non-uniformity;
  worth running with a 4 mm wide-angle lens. Recalibrate whenever the lens,
  lighting, or mounting changes.
- Histogram view — verify the gray-value spread matches the sweep metrics.

**Temperature.** Noise rises near the top of the camera's temperature range;
run sweeps at operating temperature (camera warm, not freshly powered).
