#pragma once

#include <opencv2/opencv.hpp>
#include <optional>
#include <vector>
#include "camera/camera_calib.h"
#include "experiment/trial_state.h"
#include "experiment/arena_config.h"
#include "vision/arena_mask.h"
#include "vision/camera_frame.h"
#include "vision/object_associator.h"
#include "vision/tracking_frame.h"

struct TrackingProcessOutput {
	TrackingFrame frame;
	cv::Mat display_frame;
	std::vector<std::vector<cv::Point>> contours;
};

// MOG2 + contour + Kalman pipeline (logic formerly in FerretTracker).
class TrackingPipeline {
public:
	explicit TrackingPipeline(int warmup_frames, float gsd_mm_px, float fps,
		std::optional<CameraCalib> calib = std::nullopt,
		std::optional<ArenaMaskConfig> mask_cfg = std::nullopt,
		std::optional<VisionConfig> vision_cfg = std::nullopt);

	TrackingProcessOutput process(const CameraFrame& input, TrialPhase trial_phase);

	uint64_t frame_count() const { return frame_count_; }

private:
	int warmup_frames_;
	float gsd_mm_px_;
	float fps_;
	bool use_undistort_ = false;
	cv::Mat undist_map1_;
	cv::Mat undist_map2_;
	cv::Mat undist_buf_;
	// Reused across process() calls — cv::Mat::create() (called internally by
	// bg_->apply()) is a no-op when the buffer already has the right
	// size/type, so this avoids a heap alloc/dealloc every frame at ~200fps.
	cv::Mat mask_;
	cv::Ptr<cv::BackgroundSubtractorMOG2> bg_;
	cv::KalmanFilter kf_ferret_;
	cv::KalmanFilter kf_prey_;
	cv::Mat morph_kernel_;
	ArenaMask arena_mask_;
	ObjectAssociator associator_;
	TrackState ferret_prior_;
	TrackState prey_prior_;
	uint64_t frame_count_ = 0;
	int ferret_miss_streak_ = 0;
	int prey_miss_streak_ = 0;

	void update_track(cv::KalmanFilter& kf, const std::vector<cv::Point>& contour,
		TrackState& state);
	// predict()-only fallback for a frame with no matching detection — keeps
	// the filter's internal state advancing with real time instead of
	// freezing, so a later correct() doesn't see several frames' worth of
	// motion compressed into one predict() step. Bounded by max_miss_frames
	// so a genuinely-gone track eventually reports invalid instead of
	// coasting forever.
	void coast_track(cv::KalmanFilter& kf, const TrackState& prior,
		TrackState& state, int miss_streak, int max_miss_frames);
};
