#pragma once

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <opencv2/opencv.hpp>
#include <mutex>
#include <optional>
#include <vector>
#include "camera_calib.h"
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

	// warmup_frames/gsd_mm_px overridable for calibration tests; calib.npz enables undistort.
	explicit FerretTracker(bool enable_display = false,
		int warmup_frames = WARMUP_FRAMES,
		float gsd_mm_px = GSD_MM_PX,
		std::optional<CameraCalib> calib = std::nullopt);

	void OnImageGrabbed(Pylon::CInstantCamera& camera,
		const Pylon::CGrabResultPtr& result) override;

	// Copies the latest snapshot for overlay rendering (display thread only).
	bool get_display_snapshot(DisplaySnapshot& out);

protected:
	// Frame counter exposed to test subclasses (latency suite tags CSV rows).
	uint64_t frame_count_ = 0;

private:
	bool enable_display_ = false;
	int warmup_frames_ = WARMUP_FRAMES;
	float gsd_mm_px_ = GSD_MM_PX;
	bool use_undistort_ = false;
	cv::Mat undist_map1_;
	cv::Mat undist_map2_;
	cv::Mat undist_buf_;
	cv::Ptr<cv::BackgroundSubtractorMOG2> bg_;
	cv::KalmanFilter kf_ferret_;
	cv::KalmanFilter kf_prey_;
	cv::Mat morph_kernel_;

	std::mutex display_mutex_;
	DisplaySnapshot display_snapshot_;
	bool display_ready_ = false;

	void update_track(cv::KalmanFilter& kf,
		const std::vector<cv::Point>& contour,
		TrackState& state);

	void publish_display_snapshot(const cv::Mat& frame,
		const std::vector<std::vector<cv::Point>>& contours);
};
