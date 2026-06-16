#pragma once

#include <atomic>
#include <string>

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

#include "tracker/ferret_tracker.h"

// Shared two-object measurement loop for the latency (suite 2) and mounting
// height (suite 3) calibration tools. Runs the production FerretTracker
// pipeline and records per-frame measurements + grab-to-distance latency.

struct MeasurementOptions {
	double duration_s = 30.0;          // recording window after warmup
	double warmup_s = 30.0;            // empty-arena background learning
	float gsd_mm_px = GSD_MM_PX;       // mm/px; suite 3 rescales per height
	double height_cm = -1.0;           // >= 0 adds a height_cm CSV column
	int annotate_every_n = 0;          // save annotated still every N frames (0 = off)
	std::string session_dir;           // output folder (must exist)
};

// Blocks until duration elapses or keep_running goes false.
// Writes frames.csv + summary.csv (+ stills/ when annotating) to session_dir.
// Returns 0 on success.
int run_measurement(Pylon::CBaslerUniversalInstantCamera& camera,
	const MeasurementOptions& opts,
	const std::atomic<bool>& keep_running);
