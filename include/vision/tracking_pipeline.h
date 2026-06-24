#pragma once

#include <opencv2/opencv.hpp>
#include <optional>
#include <vector>
#include "camera/camera_calib.h"
#include "experiment/trial_state.h"
#include "vision/camera_frame.h"
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
		std::optional<CameraCalib> calib = std::nullopt);

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
	cv::Ptr<cv::BackgroundSubtractorMOG2> bg_;
	cv::KalmanFilter kf_ferret_;
	cv::KalmanFilter kf_prey_;
	cv::Mat morph_kernel_;
	uint64_t frame_count_ = 0;

	void update_track(cv::KalmanFilter& kf, const std::vector<cv::Point>& contour,
		TrackState& state);
};
