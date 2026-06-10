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

After a pass, copy the `applied_settings` block into `src/camera_config.json`.

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
| `exposure_extended_sweep.json` | Exposure 19 µs–5000 µs (common mode) |
| `ultra_short_exposure_sweep.json` | Common + UltraShort exposure presets |
| `gain_sweep.json` | Gain 0–24 dB |
| `frame_rate_sweep.json` | Target fps cap |
| `frame_rate_enable_sweep.json` | Capped vs free-run max fps |
| `resolution_sweep.json` | 16 AOI width×height×offset combos |
| `offset_x_sweep.json` / `offset_y_sweep.json` | AOI centering |
| `black_level_sweep.json` | Black level 0–64 |
| `gamma_sweep.json` | Gamma 0.5–2.0 |
| `scaling_sweep.json` | In-camera scaling (requires binning off) |
| `binning_sweep.json` | Sensor vs FPGA binning 1×1–4×4 |
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
			"label": "production_1920x960",
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
		}
	]
}
```

2×2 binning doubles effective pixel size — rescale GSD and re-run
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
