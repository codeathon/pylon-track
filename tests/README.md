# Camera Calibration Test Suite

Hardware-in-the-loop tools to find the best `camera_config.json` values for the
Basler a2A1920-160umPRO arena camera. These are **not unit tests** — every tool
needs the camera attached and runs on the lab Linux box.

Built automatically with the main project (disable with
`cmake -DBUILD_CALIBRATION_TESTS=OFF ..`). Binaries land in `build/bin/`.
All outputs go to `tests/output/<suite>/<timestamp>_<label>/` (gitignored).

| Tool | Purpose |
|------|---------|
| `test_param_sweep` | Sweep one config parameter, capture labeled images + quality metrics |
| `test_resolution_sweep` | Sweep AOI width×height presets; compare FOV (mm), fps, and image quality |
| `test_latency` | Two-object tracking benchmark: speeds, centroids, distance, latency |
| `test_mount_height` | Per-height resolution check: annotated stills + the latency benchmark |

---

## Suite 1 — Parameter sweep (`test_param_sweep`)

Holds every setting at the `camera_config.json` baseline and steps **one**
parameter through the values in a sweep spec:

```bash
./bin/test_param_sweep --sweep ../tests/sweep_configs/exposure_sweep.json
```

Spec format (`tests/sweep_configs/*.json`):

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

## Suite 2 — Resolution / AOI sweep (`test_resolution_sweep`)

Steps through **width × height** (and optional `offset_x` / `offset_y`) presets.
Cropping changes **field of view in mm** and achievable **fps**, not mm/px per
pixel (GSD is set by lens + mount height). Use this to pick an AOI that covers
enough arena while sustaining the target frame rate.

```bash
./bin/test_resolution_sweep --sweep ../tests/sweep_configs/resolution_sweep.json
```

Spec format (`tests/sweep_configs/resolution_sweep.json`):

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

## Suite 3 — Two-object latency benchmark (`test_latency`)

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

## Suite 4 — Mounting height validation (`test_mount_height`)

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
