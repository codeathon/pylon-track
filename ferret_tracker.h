#pragma once

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <vector>
#include "tracker.h"

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

	explicit FerretTracker(bool enable_display = false);

	void OnImageGrabbed(Pylon::CInstantCamera& camera,
		const Pylon::CGrabResultPtr& result) override;

	// Copies the latest snapshot for overlay rendering (display thread only).
	bool get_display_snapshot(DisplaySnapshot& out);

private:
	bool enable_display_ = false;
	cv::Ptr<cv::BackgroundSubtractorMOG2> bg_;
	cv::KalmanFilter kf_ferret_;
	cv::KalmanFilter kf_prey_;
	cv::Mat morph_kernel_;
	uint64_t frame_count_ = 0;

	std::mutex display_mutex_;
	DisplaySnapshot display_snapshot_;
	bool display_ready_ = false;

	void update_track(cv::KalmanFilter& kf,
		const std::vector<cv::Point>& contour,
		TrackState& state);

	void publish_display_snapshot(const cv::Mat& frame,
		const std::vector<std::vector<cv::Point>>& contours);
};
