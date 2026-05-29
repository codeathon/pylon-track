# pylon-track

Real-time dual-animal tracking from a Basler USB3 camera using the [Basler pylon SDK](https://www.baslerweb.com/en/software/pylon/) and OpenCV. Designed for overhead arena imaging: track a **ferret** and **prey** (mouse) at ~200 fps, output position, speed, heading, and inter-animal distance in millimeters.

**Platform:** Linux only (tested workflow targets Ubuntu with USB3 Basler cameras).

## What it does

1. Opens the first available Basler camera (`CBaslerUniversalInstantCamera`).
2. Configures mono8 capture, AOI, exposure, gain, and ~200 fps frame rate (`src/camera_config.cpp`).
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
│   ├── display.h
│   ├── ferret_tracker.h
│   └── tracker.h
├── src/                      Implementation
│   ├── main.cpp              Entry point, signal handling, stdout loop
│   ├── camera_config.cpp     Basler GenICam settings (AOI, exposure, FPS)
│   ├── ferret_tracker.cpp    Pylon image handler: BG subtract + track logic
│   ├── tracker.cpp           Shared Kalman filter helper
│   └── display.cpp           Live overlay window (helper thread)
└── build/                    Out-of-source build (created by you)
    └── bin/
        └── ferret_tracker    Executable output
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
sudo apt install -y build-essential cmake libopencv-dev
```

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
./build/bin/ferret_tracker
```

Headless mode (default): telemetry only on stdout.

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

Tuned in `src/camera_config.cpp` and `include/ferret_tracker.h`:

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

## CMake targets

| Target | Command | Description |
|--------|---------|-------------|
| `ferret_tracker` | `make ferret_tracker` | Build executable |
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

## License

Add a license file if you plan to distribute this project publicly.
