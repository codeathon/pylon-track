#include "tracker/ferret_tracker.h"
#include "log/logger.h"
#include <cmath>
#include <chrono>
#include <algorithm>
#include <sstream>

FerretTracker::FerretTracker(bool enable_display, int warmup_frames, float gsd_mm_px,
	std::optional<CameraCalib> calib)
	: enable_display_(enable_display)
	, warmup_frames_(warmup_frames)
	, gsd_mm_px_(gsd_mm_px)
	, bg_(cv::createBackgroundSubtractorMOG2(500, 16, false))
	, kf_ferret_(make_kalman(FPS))
	, kf_prey_(make_kalman(FPS))
	// 7px kernel ≈ 7mm — removes debris smaller than a mouse paw
	, morph_kernel_(cv::getStructuringElement(cv::MORPH_ELLIPSE, {7, 7}))
{
	if (calib && calib->enabled()) {
		use_undistort_ = true;
		undist_map1_ = calib->map1;
		undist_map2_ = calib->map2;
		undist_buf_ = cv::Mat(calib->image_size, CV_8UC1);
		log_info("tracker", "Lens undistort enabled");
	}
}

bool FerretTracker::get_display_snapshot(DisplaySnapshot& out) {
	if (!enable_display_) {
		return false;
	}
	std::lock_guard<std::mutex> lock(display_mutex_);
	if (!display_ready_) {
		return false;
	}
	out = display_snapshot_;
	return true;
}

void FerretTracker::publish_display_snapshot(const cv::Mat& frame,
	const std::vector<std::vector<cv::Point>>& contours)
{
	std::lock_guard<std::mutex> lock(display_mutex_);
	display_snapshot_.frame = frame.clone();
	display_snapshot_.contours = contours;
	display_snapshot_.ferret = ferret;
	display_snapshot_.prey = prey;
	display_snapshot_.frame_count = frame_count_;
	display_snapshot_.warmup = frame_count_ < static_cast<uint64_t>(warmup_frames_);
	display_ready_ = true;
}

void FerretTracker::OnImageGrabbed(Pylon::CInstantCamera&,
	const Pylon::CGrabResultPtr& result)
{
	if (!result->GrabSucceeded()) return;

	auto t0 = std::chrono::steady_clock::now();

	// Zero-copy wrap of the pylon buffer; remap into undist_buf_ when calib.npz is loaded.
	cv::Mat frame_raw(result->GetHeight(), result->GetWidth(),
		CV_8UC1, result->GetBuffer());
	cv::Mat frame;
	if (use_undistort_) {
		cv::remap(frame_raw, undist_buf_, undist_map1_, undist_map2_, cv::INTER_LINEAR);
		frame = undist_buf_;
	} else {
		frame = frame_raw;
	}

	// Background subtraction
	// Faster learning during warmup, slow during experiment to avoid adapting to animals
	const double lr = (frame_count_ < static_cast<uint64_t>(warmup_frames_)) ? 0.01 : 0.002;
	cv::Mat mask;
	bg_->apply(frame, mask, lr);

	// Morphological opening: removes small noise blobs
	cv::morphologyEx(mask, mask, cv::MORPH_OPEN, morph_kernel_);

	// Find foreground blobs
	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	// Filter by area:
	//   min 200px²  ≈ mouse paw (noise floor)
	//   max 60000px² ≈ ferret fully visible
	contours.erase(
		std::remove_if(contours.begin(), contours.end(), [](const auto& c) {
			float a = cv::contourArea(c);
			return a < 200.0f || a > 60000.0f;
		}), contours.end());

	// Sort largest first: contours[0] = ferret, contours[1] = prey
	std::sort(contours.begin(), contours.end(), [](const auto& a, const auto& b) {
		return cv::contourArea(a) > cv::contourArea(b);
	});

	if (contours.size() >= 2) {
		// Normal: two distinct blobs visible
		update_track(kf_ferret_, contours[0], ferret);
		update_track(kf_prey_,   contours[1], prey);
	} else if (contours.size() == 1) {
		float area = cv::contourArea(contours[0]);
		if (area > 15000.0f) {
			// Ferret and prey merged (capture/occlusion event)
			// Update ferret with detection; coast prey on Kalman prediction only
			update_track(kf_ferret_, contours[0], ferret);
			cv::Mat pred = kf_prey_.predict();
			prey.pos_px       = {pred.at<float>(0), pred.at<float>(1)};
			prey.pos_mm       = prey.pos_px * gsd_mm_px_;
			prey.speed_mm_s   = std::sqrt(std::pow(pred.at<float>(2), 2) +
				std::pow(pred.at<float>(3), 2)) * FPS * gsd_mm_px_;
			prey.direction_deg = std::atan2(-pred.at<float>(3), pred.at<float>(2))
				* 180.0f / M_PI;
			// prey.valid stays true — coasting on prior velocity
		} else {
			// Only ferret visible, prey out of frame or hidden
			update_track(kf_ferret_, contours[0], ferret);
			prey.valid = false;
		}
	} else {
		ferret.valid = false;
		prey.valid   = false;
	}

	if (enable_display_) {
		publish_display_snapshot(frame, contours);
	}

	++frame_count_;

	// Log frame processing time ~1 Hz at 200 fps (Debug only).
	if (frame_count_ % 200 == 0) {
		const auto dt_us = std::chrono::duration_cast<std::chrono::microseconds>(
			std::chrono::steady_clock::now() - t0).count();
		std::ostringstream oss;
		oss << "OnImageGrabbed " << dt_us << " us";
		log_debug("tracker", oss.str());
	}
}

void FerretTracker::update_track(cv::KalmanFilter& kf,
	const std::vector<cv::Point>& contour,
	TrackState& state)
{
	cv::Moments m = cv::moments(contour);
	cv::Point2f px = {(float)(m.m10 / m.m00), (float)(m.m01 / m.m00)};

	kf.predict();
	cv::Mat meas = (cv::Mat_<float>(2, 1) << px.x, px.y);
	cv::Mat corr = kf.correct(meas);

	float vx_px = corr.at<float>(2);
	float vy_px = corr.at<float>(3);

	state.pos_px       = {corr.at<float>(0), corr.at<float>(1)};
	state.pos_mm       = state.pos_px * gsd_mm_px_;
	state.speed_mm_s   = std::sqrt(vx_px * vx_px + vy_px * vy_px) * FPS * gsd_mm_px_;
	state.direction_deg = std::atan2(-vy_px, vx_px) * 180.0f / M_PI; // image Y flipped
	state.valid        = true;
}
