#pragma once

#include <atomic>
#include <string>

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

#include "tracker/ferret_tracker.h"

// Runs the live telemetry loop: prints speed, heading, and inter-object
// distance to stdout once per valid data pair, prefixed with session
// elapsed time and data-arrival interval/rate.
//
// Output format (one line per valid frame pair, ~200 Hz):
//   t=<ms>ms  dt=<ms>ms(<Hz>Hz)  Ferret: ...  |  Prey: ...  |  Dist: <mm>mm
//
// Blocks until keep_running goes false. Returns 0 on success.

struct TelemetryOptions {
    double warmup_s  = 30.0;       // background model learning period
    float  gsd_mm_px = GSD_MM_PX;  // mm per pixel (override for non-default heights)
};

int run_telemetry(Pylon::CBaslerUniversalInstantCamera& camera,
    const TelemetryOptions& opts,
    const std::atomic<bool>& keep_running);
