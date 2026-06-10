# pylon-track

Real-time dual-animal tracking from a Basler USB3 camera using the [Basler pylon SDK](https://www.baslerweb.com/en/software/pylon/) and OpenCV. Designed for overhead arena imaging: track a **ferret** and **prey** (mouse) at ~200 fps, output position, speed, heading, and inter-animal distance in millimeters.

**Platform:** Linux only (tested workflow targets Ubuntu with USB3 Basler cameras).

A companion **calibration test suite** under [`tests/`](tests/) sweeps camera settings,
benchmarks two-object tracking latency, and validates mounting height — see
[Calibration and testing](#calibration-and-testing).

## What it does

1. Opens the first available Basler camera (`CBaslerUniversalInstantCamera`).
2. Configures mono8 capture, AOI, exposure, gain, and frame rate from [`src/camera_config.json`](src/camera_config.json).
3. On each frame (`src/ferret_tracker.cpp`):
   - MOG2 background subtraction (30 s warmup — keep arena empty)
   - Morphological cleanup + contour detection
   - Assigns largest blob → ferret, second largest → prey
   - Kalman filtering per animal for smooth position/velocity
   - Handles merge/occlusion (single large blob) by coasting prey on prediction
4. Prints live telemetry to stdout until `Ctrl+C` or `SIGTERM`.

Example line:

```text
Ferret: (450, 320)mm  850mm/s  45deg  |  Prey: (520, 410)mm  120mm/s  90deg  |  Dist: 120mm
```

## Repository layout

```text
pylon-track/
├── CMakeLists.txt
├── include/                  Public headers
│   ├── camera_config.h
│   ├── camera_settings.h
│   ├── display.h
│   ├── ferret_tracker.h
│   ├── logger.h                Global thread-safe logger
│   └── tracker.h
├── src/                      Implementation
│   ├── main.cpp              Entry point, signal handling, stdout loop
│   ├── camera_config.cpp     Load JSON + apply GenICam settings
│   ├── camera_config.json    Camera exposure, gain, AOI, frame rate (edit this)
│   ├── ferret_tracker.cpp    Pylon image handler: BG subtract + track logic
│   ├── tracker.cpp           Shared Kalman filter helper
│   ├── display.cpp           Live overlay window (helper thread)
│   └── logger.cpp
├── tests/                    Camera calibration suite (hardware-in-the-loop)
│   ├── README.md             Full run protocols + Basler calibration notes
│   ├── common/               Shared: session dirs, CSV writer, image metrics
│   ├── sweep_configs/        Example parameter sweep specs (JSON)
│   ├── param_sweep.cpp       Suite 1 → test_param_sweep
│   ├── resolution_sweep.cpp  Suite 2 → test_resolution_sweep
│   ├── latency_suite.cpp     Suite 3 → test_latency
│   └── mount_height_suite.cpp Suite 4 → test_mount_height
└── build/                    Out-of-source build (created by you)
    └── bin/
        ├── ferret_tracker      Production tracker
        ├── test_param_sweep        Calibration: config parameter sweep
        ├── test_resolution_sweep   Calibration: AOI / resolution preset sweep
        ├── test_latency            Calibration: two-object latency benchmark
        ├── test_mount_height       Calibration: mounting height validation
        └── camera_config.json  Copied from src/ at build time
```

Source files still use the `ferret_tracker` name internally; the repo name reflects the **pylon** camera stack.

## Requirements

### Required installations

All four are required before `cmake` will succeed:

| Package | Purpose | Ubuntu / Debian package |
|---------|---------|-------------------------|
| **build-essential** | C++17 compiler, `make`, linker | `build-essential` |
| **cmake** | Configure the project (≥ 3.16) | `cmake` |
| **OpenCV (dev)** | MOG2, contours, Kalman (`core`, `imgproc`, `video`) | `libopencv-dev` |
| **Basler pylon SDK** | Camera grab + GenICam API | [Install from Basler](https://www.baslerweb.com/en/software/pylon/) → default `/opt/pylon` |

### Hardware

- Basler **USB3** camera (settings target **a2A1920-160umPRO**-class sensor at 1.2 m, 4 mm lens)
- USB3 port with enough bandwidth for ~200 fps mono8

## Install dependencies

### Ubuntu / Debian

```bash
sudo apt update
sudo apt install -y build-essential cmake libopencv-dev nlohmann-json3-dev
```

`nlohmann-json3-dev` is optional but avoids CMake downloading json during configure.

### Basler pylon SDK

1. Download and install the **pylon Camera Software Suite** for Linux from [Basler pylon](https://www.baslerweb.com/en/software/pylon/).
2. Default install path is `/opt/pylon` (used by `CMakeLists.txt`).
3. Confirm headers exist:

```bash
ls /opt/pylon/include/pylon/PylonIncludes.h
```

If installed elsewhere, pass `-DPYLON_ROOT=/your/pylon/path` when running `cmake`.

### Verify OpenCV (dev)

CMake needs `OpenCVConfig.cmake` (from the **dev** package, not runtime-only):

```bash
ls /usr/lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake
```

## Build

### Configure and compile

```bash
cd pylon-track
mkdir -p build && cd build
cmake -DPYLON_ROOT=/opt/pylon ..
make
```

This builds `ferret_tracker` and the calibration tools (`test_param_sweep`,
`test_resolution_sweep`, `test_latency`, `test_mount_height`). Disable the suite with
`-DBUILD_CALIBRATION_TESTS=OFF` if you only need the production binary.

If pylon is installed elsewhere:

```bash
cmake -DPYLON_ROOT=/path/to/pylon ..
```

If OpenCV is installed but CMake still cannot find it:

```bash
cmake -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4 -DPYLON_ROOT=/opt/pylon ..
```

## Run

```bash
./build/bin/ferret_tracker
```

Headless mode (default): status messages use the global logger; tracking telemetry prints as **raw** lines (no timestamp prefix) for easy piping.

### Camera configuration (`src/camera_config.json`)

Edit [`src/camera_config.json`](src/camera_config.json) to tune brightness and imaging (no rebuild required — restart the app):

| Field | Default | Unit / notes |
|-------|---------|----------------|
| `exposure_time_us` | `2000` | Microseconds (raise to brighten, e.g. `8000`) |
| `gain_db` | `6.0` | dB (raise if still dark; adds noise) |
| `exposure_auto` | `false` | `true` enables auto exposure |
| `gain_auto` | `false` | `true` enables auto gain |
| `width` / `height` | `1920` / `960` | AOI size |
| `offset_x` / `offset_y` | `0` / `120` | AOI position |
| `frame_rate_fps` | `200.0` | Target FPS |

Override config path (first match wins):

```bash
./build/bin/ferret_tracker --camera-config /path/to/camera_config.json
export PYLON_CAMERA_CONFIG=src/camera_config.json
```

Default lookup: `camera_config.json` next to the executable (`build/bin/`), then `src/camera_config.json` from repo root.

### Logging

| Flag | Effect |
|------|--------|
| (default) | `INFO` and above to stdout/stderr |
| `--verbose` | Include `DEBUG` (e.g. frame timing ~1 Hz from tracker) |
| `--log-file PATH` | Append all log lines to a file (creates parent dirs) |

Example log line:

```text
[2026-05-29 14:30:01.234] [INFO] [main] Camera: a2A1920-160umPRO
```

Telemetry lines remain unprefixed:

```text
Ferret: (450, 320)mm  850mm/s  45deg  |  Prey: (520, 410)mm  ...
```

```bash
./build/bin/ferret_tracker --verbose --log-file /tmp/pylon-track.log
```

### Live display (optional)

```bash
./build/bin/ferret_tracker --display
```

- Opens an OpenCV window on a **dedicated helper thread** (~30 Hz refresh)
- Shows the camera frame with ferret/prey contours, center markers, labels, and distance line
- Main thread continues printing stdout telemetry in parallel
- Quit: press **q** or **ESC** in the window (or `Ctrl+C` in the terminal)

Requires a graphical session (`DISPLAY` set). For SSH, use X forwarding (`ssh -X`) or run on the machine with a monitor attached.

On first launch, keep the **arena empty for 30 seconds** while the background model warms up (`WARMUP_FRAMES` in `include/ferret_tracker.h`).

### USB access (without root)

Install Basler udev rules once (from build dir):

```bash
make install_udev
```

### Low-latency / real-time scheduling (optional)

```bash
make run_rt
```

Runs the binary under `chrt -f 50` (SCHED_FIFO). May require:

```bash
sudo setcap cap_sys_nice+ep build/bin/ferret_tracker
```

## Camera and optics assumptions

Camera GenICam settings: [`src/camera_config.json`](src/camera_config.json). Tracker constants in `include/ferret_tracker.h`:

| Constant | Value | Meaning |
|----------|-------|---------|
| Resolution | 1920×960 (AOI) | Center crop for ~200 fps |
| Frame rate | 200 fps | `AcquisitionFrameRate` |
| Exposure | `exposure_time_us` in JSON (default 2000 µs) | Raise to brighten; more motion blur |
| Gain | `gain_db` in JSON (default 6 dB) | Raise if still dark; adds noise |
| `GSD_MM_PX` | 1.035 mm/px | Ground sample distance at 1.2 m mount |
| `WARMUP_FRAMES` | 6000 (~30 s) | Background learning period |

Change these if your mount height, lens, or arena size differs.

## Tracking parameters

| Parameter | Location | Purpose |
|-----------|----------|---------|
| Contour area 200–60000 px² | `src/ferret_tracker.cpp` | Noise floor / max ferret blob |
| Merge threshold 15000 px² | `src/ferret_tracker.cpp` | Single-blob = ferret+prey overlap |
| MOG2 history 500, var threshold 16 | `src/ferret_tracker.cpp` | Background model |
| BG learning rate 0.01 → 0.002 | `src/ferret_tracker.cpp` | Fast warmup, slow during experiment |
| Morph kernel 7×7 ellipse | `src/ferret_tracker.cpp` | Remove sub-paw noise |

## Architecture (data flow)

```text
Basler camera (pylon Grab)
        │
        ▼
OnImageGrabbed (zero-copy cv::Mat on frame buffer)
        │
        ├─ MOG2 background subtract
        ├─ morphology + findContours
        ├─ sort by area → ferret / prey assignment
        └─ Kalman predict/correct per track
        │
        ▼
main loop reads TrackState → printf distance + kinematics
```

## Calibration and testing

Hardware-in-the-loop tools under [`tests/`](tests/) help choose `camera_config.json`
values, quantify tracking latency, and validate camera mounting height. These are
**not automated unit tests** — each tool needs the Basler camera attached on the
lab Linux box.

Full run protocols, CSV column definitions, and Basler image-quality guidance:
[`tests/README.md`](tests/README.md).

### Why three suites

| Suite | Binary | Question it answers |
|-------|--------|---------------------|
| 1 — Parameter sweep | `test_param_sweep` | Which exposure / gain / fps gives the best image for tracking? |
| 2 — Resolution / AOI | `test_resolution_sweep` | Which crop size balances arena FOV (mm), fps, and image quality? |
| 3 — Latency benchmark | `test_latency` | How fast does grab → distance-between-objects run at fixed capture rate? |
| 4 — Mount height | `test_mount_height` | At this height, do objects still resolve (≥200 px²) and measure accurately? |

Recommended order: **suite 1** (lock exposure/gain) → **suite 2** (pick AOI) →
**suite 4** at candidate mount heights → **suite 3** at each height to confirm
latency and distance accuracy.

### Build calibration tools

Built by default with `make` (see [Build](#build)). Binaries:

```text
build/bin/test_param_sweep
build/bin/test_resolution_sweep
build/bin/test_latency
build/bin/test_mount_height
```

Outputs land in `tests/output/<suite>/<timestamp>_<label>/` (gitignored).

### Suite 1 — Parameter sweep

Holds every setting at the `camera_config.json` baseline and steps **one**
parameter through values from a sweep spec JSON:

```bash
cd build
./bin/test_param_sweep --sweep ../tests/sweep_configs/exposure_sweep.json
```

Example specs in [`tests/sweep_configs/`](tests/sweep_configs/) (`exposure_sweep.json`,
`gain_sweep.json`, `frame_rate_sweep.json`). Per run:

- `sweep.csv` — achieved fps, mean gray, stddev, clipped-pixel %, Laplacian variance
- Sample PNGs — `<timestamp>_<parameter>_<value>_fNNN.png`

Pick values where mean gray is ~128–180, clipping is near zero, and Laplacian
variance is high without excessive gain noise.

### Suite 2 — Resolution / AOI sweep

Sweeps **width × height** presets (optional offsets) and reports field of view
in mm at the configured GSD, plus achieved fps and image metrics:

```bash
./bin/test_resolution_sweep --sweep ../tests/sweep_configs/resolution_sweep.json
```

Example spec: [`tests/sweep_configs/resolution_sweep.json`](tests/sweep_configs/resolution_sweep.json).
Per run: `resolution.csv` + labeled PNGs. Override GSD with `--gsd 1.29` if
mount height differs from 1.2 m. Pick a preset that covers the full arena
while meeting the fps target; confirm blob size with suite 4.

### Suite 3 — Two-object latency benchmark

Runs the **same pipeline** as production (`FerretTracker`: MOG2 + Kalman) and
records per-frame kinematics plus grab-to-distance latency:

```bash
./bin/test_latency --duration 30 --warmup-secs 30
```

Protocol: keep the arena **empty** during warmup, then move two objects at
known speeds (slow / medium / fast across separate runs).

Per run:

- `frames.csv` — speeds, centroids (px and mm), distance (mm), `latency_us`, validity flags
- `summary.csv` — achieved fps, valid-pair %, latency mean / p50 / p95 / max

Shared flags: `--camera-config`, `--output`, `--gsd`, `--verbose`.

### Suite 4 — Mounting height validation

Operator-in-the-loop: mount the camera at a candidate height, then:

```bash
./bin/test_mount_height --height-cm 120 --duration 30
```

- Rescales GSD from the 1.2 m baseline (`1.035 mm/px × height / 120`)
- Saves annotated stills (~1 Hz) in `stills/` — contours, boxes, blob area in px²
- Runs the suite-2 measurement loop; `frames.csv` includes `height_cm`

Compare `distance_mm` in the CSV against a tape-measured separation. Repeat at
each candidate height (e.g. 100, 120, 150 cm) and compare stills for blobs
above the 200 px² tracking floor.

### Basler calibration (summary)

Distilled from [Basler image-quality docs](https://docs.baslerweb.com/optimizing-image-quality):

- **Motion blur:** `blur_px = speed_mm_s × exposure_s / GSD` — keep under ~1 px;
  at 1 m/s and 1.035 mm/px, exposure should stay under ~1 ms.
- **Brightness:** target ~50–70 % of Mono8 range; avoid clipping at 0 or 255.
- **Gain:** amplifies noise equally with signal — prefer light/exposure first.
- **Frame period:** at 200 fps, exposure must fit within ~5000 µs minus readout.
- **pylon Viewer:** use *Automatic Image Adjustment* for a baseline, then lock
  values into `camera_config.json`; *Flat-Field Correction* for vignetting with
  the 4 mm lens.

See [`tests/README.md`](tests/README.md) for the full GSD-vs-height table and
metric interpretation.

## CMake targets

| Target | Command | Description |
|--------|---------|-------------|
| `ferret_tracker` | `make ferret_tracker` | Production tracker executable |
| `test_param_sweep` | `make test_param_sweep` | Calibration: parameter sweep |
| `test_resolution_sweep` | `make test_resolution_sweep` | Calibration: AOI / resolution sweep |
| `test_latency` | `make test_latency` | Calibration: latency benchmark |
| `test_mount_height` | `make test_mount_height` | Calibration: mount height validation |
| `run_rt` | `make run_rt` | Run with SCHED_FIFO priority |
| `install_udev` | `make install_udev` | Install Basler USB udev rules |

## Troubleshooting

| Issue | Things to check |
|-------|-----------------|
| `Could not find a package configuration file provided by "OpenCV"` | Install `libopencv-dev`, then use `-DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4` if needed |
| `--display` / `cannot open display` | Run on a desktop session or `ssh -X`; ensure `echo $DISPLAY` is set |
| `chrt: Operation not permitted` (`make run_rt`) | Use `./build/bin/ferret_tracker` instead, or `sudo setcap cap_sys_nice+ep build/bin/ferret_tracker` |
| `pylon SDK not found` | Set `-DPYLON_ROOT=` to your install prefix |
| No camera found | USB cable, `install_udev`, camera powered |
| No valid tracks after warmup | Lighting, gain (target ~80–100 DN background), arena contrast |
| Low frame rate | AOI size, `DeviceLinkThroughputLimitMode`, USB3 port |
| High jitter | `make run_rt`, CPU isolation, reduce pipeline load |
| Calibration tools missing after build | Ensure `BUILD_CALIBRATION_TESTS=ON` (default); run `make` not only `make ferret_tracker` |
| `test_latency` valid-pair % low | Empty arena during warmup; two distinct moving blobs; check contour area ≥200 px² |
| Sweep images all dark / clipped | Adjust `tests/sweep_configs/` ranges; brighten arena lighting before sweeping gain |

## License

Add a license file if you plan to distribute this project publicly.
