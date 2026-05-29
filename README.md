# pylon-track

Real-time dual-animal tracking from a Basler USB3 camera using the [Basler pylon SDK](https://www.baslerweb.com/en/software/pylon/) and OpenCV. Designed for overhead arena imaging: track a **ferret** and **prey** (mouse) at ~200 fps, output position, speed, heading, and inter-animal distance in millimeters.

**Platform:** Linux only (tested workflow targets Ubuntu with USB3 Basler cameras).

## What it does

1. Opens the first available Basler camera (`CBaslerUniversalInstantCamera`).
2. Configures mono8 capture, AOI, exposure, gain, and ~200 fps frame rate (`camera_config.cpp`).
3. On each frame (`ferret_tracker.cpp`):
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
├── main.cpp              Entry point, signal handling, stdout loop
├── camera_config.cpp     Basler GenICam settings (AOI, exposure, FPS)
├── ferret_tracker.cpp    Pylon image handler: BG subtract + track logic
├── ferret_tracker.h
├── tracker.cpp           Shared Kalman filter helper
├── tracker.h             TrackState struct
└── CMakeLists.txt        Build + optional run_rt / install_udev targets
```

Source files still use the `ferret_tracker` name internally; the repo name reflects the **pylon** camera stack.

## Requirements

| Dependency | Notes |
|------------|--------|
| **Basler pylon SDK** | Default path `/opt/pylon`. [Download](https://www.baslerweb.com/en/software/pylon/) |
| **OpenCV** | `core`, `imgproc`, `video` (MOG2, contours, Kalman). Install dev headers (see below). |
| **CMake** | ≥ 3.16 |
| **C++ compiler** | C++17 |
| **Hardware** | Basler USB3 camera (configured for **a2A1920-160umPRO**-class sensor at 1.2 m, 4 mm lens) |

## Build

### 1. Install OpenCV (required)

CMake needs the **development** package (includes `OpenCVConfig.cmake`), not just runtime libraries.

**Ubuntu / Debian:**

```bash
sudo apt update
sudo apt install -y libopencv-dev build-essential cmake
```

**Fedora:**

```bash
sudo dnf install -y opencv-devel cmake gcc-c++
```

Verify the config file exists:

```bash
ls /usr/lib/x86_64-linux-gnu/cmake/opencv4/OpenCVConfig.cmake
```

### 2. Configure and compile

```bash
cd pylon-track
mkdir -p build && cd build
cmake -DPYLON_ROOT=/opt/pylon ..
make ferret_tracker
```

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
./build/ferret_tracker
```

On first launch, keep the **arena empty for 30 seconds** while the background model warms up (`WARMUP_FRAMES` in `ferret_tracker.h`).

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
sudo setcap cap_sys_nice+ep build/ferret_tracker
```

## Camera and optics assumptions

Tuned in `camera_config.cpp` and `ferret_tracker.h`:

| Constant | Value | Meaning |
|----------|-------|---------|
| Resolution | 1920×960 (AOI) | Center crop for ~200 fps |
| Frame rate | 200 fps | `AcquisitionFrameRate` |
| Exposure | 2000 µs | Limits motion blur |
| `GSD_MM_PX` | 1.035 mm/px | Ground sample distance at 1.2 m mount |
| `WARMUP_FRAMES` | 6000 (~30 s) | Background learning period |

Change these if your mount height, lens, or arena size differs.

## Tracking parameters

| Parameter | Location | Purpose |
|-----------|----------|---------|
| Contour area 200–60000 px² | `ferret_tracker.cpp` | Noise floor / max ferret blob |
| Merge threshold 15000 px² | `ferret_tracker.cpp` | Single-blob = ferret+prey overlap |
| MOG2 history 500, var threshold 16 | `ferret_tracker.cpp` | Background model |
| BG learning rate 0.01 → 0.002 | `ferret_tracker.cpp` | Fast warmup, slow during experiment |
| Morph kernel 7×7 ellipse | `ferret_tracker.cpp` | Remove sub-paw noise |

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

## CMake targets

| Target | Command | Description |
|--------|---------|-------------|
| `ferret_tracker` | `make ferret_tracker` | Build executable |
| `run_rt` | `make run_rt` | Run with SCHED_FIFO priority |
| `install_udev` | `make install_udev` | Install Basler USB udev rules |

## Troubleshooting

| Issue | Things to check |
|-------|-----------------|
| `Could not find a package configuration file provided by "OpenCV"` | Install `libopencv-dev` (Ubuntu) or `opencv-devel` (Fedora). Then use `-DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4` if needed |
| `pylon SDK not found` | Set `-DPYLON_ROOT=` to your install prefix |
| No camera found | USB cable, `install_udev`, camera powered |
| No valid tracks after warmup | Lighting, gain (target ~80–100 DN background), arena contrast |
| Low frame rate | AOI size, `DeviceLinkThroughputLimitMode`, USB3 port |
| High jitter | `make run_rt`, CPU isolation, reduce pipeline load |

## License

Add a license file if you plan to distribute this project publicly.
