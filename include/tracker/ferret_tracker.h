#pragma once

#include <pylon/PylonIncludes.h>
#include <opencv2/opencv.hpp>
#include <optional>
#include "camera/camera_calib.h"
#include "experiment/session_recorder.h"
#include "experiment/trial_state.h"
#include "tracker/tracker.h"
#include "experiment/arena_config.h"
#include "vision/arena_mask.h"
#include "vision/camera_tracking_service.h"
#include "vision/tracking_frame.h"

// Camera / optics constants for a2A1920-160umPRO at 1.2m with 4mm lens
static constexpr float GSD_MM_PX = 1.035f;
static constexpr float FPS       = 200.0f;
static constexpr int   WARMUP_FRAMES = static_cast<int>(FPS * 30);

// Thin wrapper over CameraTrackingService — keeps test/latency API stable.
class FerretTracker : public CameraTrackingService {
public:
	explicit FerretTracker(bool enable_display = false,
		int warmup_frames = WARMUP_FRAMES,
		float gsd_mm_px = GSD_MM_PX,
		std::optional<CameraCalib> calib = std::nullopt,
		std::optional<ArenaMaskConfig> mask = std::nullopt,
		std::optional<VisionConfig> vision = std::nullopt);

	void set_experiment_hooks(TrialStateMachine* fsm, SessionRecorder* recorder);
};
