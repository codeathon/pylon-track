#pragma once

#include <cstdint>
#include "experiment/trial_state.h"
#include "tracker.h"

struct AnimalTrack {
	TrackState state;
};

// Output of OpenCV tracking — input to chase policy and session recorder.
struct TrackingFrame {
	uint64_t frame_index = 0;
	uint64_t camera_ts_ticks = 0;
	int64_t host_time_ns = 0;

	AnimalTrack ferret;
	AnimalTrack prey;

	float distance_mm = -1.0f;
	float bearing_deg = 0.0f;
	float closing_speed_mm_s = 0.0f;

	TrialPhase trial_phase = TrialPhase::Warmup;
	bool warmup = false;

	bool both_valid() const;
};

// Recompute distance / bearing / closing speed from track states.
void fill_tracking_derived(TrackingFrame& frame, float fps);
