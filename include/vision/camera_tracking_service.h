#pragma once

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <mutex>
#include <optional>
#include <vector>
#include "camera/camera_calib.h"
#include "experiment/session_recorder.h"
#include "experiment/trial_state.h"
#include "tracker/tracker.h"
#include "vision/tracking_frame.h"
#include "vision/tracking_pipeline.h"

// Thread-safe frame + tracks for the display helper thread.
struct DisplaySnapshot {
	cv::Mat frame;
	std::vector<std::vector<cv::Point>> contours;
	TrackState ferret;
	TrackState prey;
	uint64_t frame_count = 0;
	bool warmup = false;
};

// Pylon grab callback + TrackingPipeline — no experiment-specific orchestration.
class CameraTrackingService : public Pylon::CImageEventHandler {
public:
	struct Options {
		int warmup_frames = 6000;
		float gsd_mm_px = 1.035f;
		float fps = 200.0f;
		std::optional<CameraCalib> calib;
		std::optional<ArenaMaskConfig> mask;
		std::optional<VisionConfig> vision;
		bool enable_display = false;
	};

	explicit CameraTrackingService(const Options& opts);

	void set_trial_fsm(TrialStateMachine* fsm) { trial_fsm_ = fsm; }
	void set_session_recorder(SessionRecorder* recorder) { session_recorder_ = recorder; }

	void OnImageGrabbed(Pylon::CInstantCamera& camera,
		const Pylon::CGrabResultPtr& result) override;

	bool get_display_snapshot(DisplaySnapshot& out);
	bool get_tracking_frame(TrackingFrame& out);

	TrackState ferret;
	TrackState prey;
	uint64_t frame_count() const { return frame_count_; }

protected:
	uint64_t frame_count_ = 0;

private:
	Options opts_;
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
