#pragma once

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <optional>
#include <vector>
#include "camera/camera_calib.h"
#include "experiment/session_recorder.h"
#include "experiment/trial_state.h"
#include "tracker/tracker.h"
#include "vision/tracking_frame.h"
#include "vision/tracking_pipeline.h"

// Camera / optics constants for a2A1920-160umPRO at 1.2m with 4mm lens
static constexpr float GSD_MM_PX = 1.035f;  // mm per pixel
static constexpr float FPS       = 200.0f;
static constexpr int   WARMUP_FRAMES = static_cast<int>(FPS * 30); // 30s BG warmup

// Thread-safe frame + tracks for the display helper thread.
struct DisplaySnapshot {
	cv::Mat frame;
	std::vector<std::vector<cv::Point>> contours;
	TrackState ferret;
	TrackState prey;
	uint64_t frame_count = 0;
	bool warmup = false;
};

class FerretTracker : public Pylon::CImageEventHandler {
public:
	TrackState ferret;
	TrackState prey;

	// warmup_frames/gsd_mm_px overridable for calibration tests; calib.npz enables undistort.
	explicit FerretTracker(bool enable_display = false,
		int warmup_frames = WARMUP_FRAMES,
		float gsd_mm_px = GSD_MM_PX,
		std::optional<CameraCalib> calib = std::nullopt);

	// Optional Phase 0 hooks — nullptr disables experiment CSV / trial phases.
	void set_experiment_hooks(TrialStateMachine* fsm, SessionRecorder* recorder);

	void OnImageGrabbed(Pylon::CInstantCamera& camera,
		const Pylon::CGrabResultPtr& result) override;

	// Copies the latest snapshot for overlay rendering (display thread only).
	bool get_display_snapshot(DisplaySnapshot& out);

	// Mutex-protected copy of the latest TrackingFrame (motor / telemetry consumers).
	bool get_tracking_frame(TrackingFrame& out);

protected:
	// Frame counter exposed to test subclasses (latency suite tags CSV rows).
	uint64_t frame_count_ = 0;

private:
	bool enable_display_ = false;
	int warmup_frames_ = WARMUP_FRAMES;
	TrackingPipeline pipeline_;

	TrialStateMachine* trial_fsm_ = nullptr;
	SessionRecorder* session_recorder_ = nullptr;

	std::mutex display_mutex_;
	DisplaySnapshot display_snapshot_;
	bool display_ready_ = false;

	std::mutex tracking_mutex_;
	TrackingFrame latest_tracking_;
	bool tracking_ready_ = false;

	void publish_display_snapshot(const cv::Mat& frame,
		const std::vector<std::vector<cv::Point>>& contours,
		bool warmup);
};
