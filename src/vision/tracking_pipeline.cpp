#include "vision/tracking_pipeline.h"

#include <algorithm>
#include <cmath>
#include "log/logger.h"
#include "tracker.h"

TrackingPipeline::TrackingPipeline(int warmup_frames, float gsd_mm_px, float fps,
	std::optional<CameraCalib> calib)
	: warmup_frames_(warmup_frames)
	, gsd_mm_px_(gsd_mm_px)
	, fps_(fps)
	, bg_(cv::createBackgroundSubtractorMOG2(500, 16, false))
	, kf_ferret_(make_kalman(fps))
	, kf_prey_(make_kalman(fps))
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

void TrackingPipeline::update_track(cv::KalmanFilter& kf,
	const std::vector<cv::Point>& contour, TrackState& state)
{
	cv::Moments m = cv::moments(contour);
	cv::Point2f px = {static_cast<float>(m.m10 / m.m00),
		static_cast<float>(m.m01 / m.m00)};

	kf.predict();
	cv::Mat meas = (cv::Mat_<float>(2, 1) << px.x, px.y);
	cv::Mat corr = kf.correct(meas);

	const float vx_px = corr.at<float>(2);
	const float vy_px = corr.at<float>(3);

	state.pos_px = {corr.at<float>(0), corr.at<float>(1)};
	state.pos_mm = state.pos_px * gsd_mm_px_;
	state.speed_mm_s = std::sqrt(vx_px * vx_px + vy_px * vy_px) * fps_ * gsd_mm_px_;
	state.direction_deg = std::atan2(-vy_px, vx_px) * 180.0f / static_cast<float>(M_PI);
	state.valid = true;
}

TrackingProcessOutput TrackingPipeline::process(const CameraFrame& input,
	TrialPhase trial_phase)
{
	TrackingProcessOutput out;
	out.frame.frame_index = input.frame_index;
	out.frame.camera_ts_ticks = input.camera_ts_ticks;
	out.frame.host_time_ns = input.host_time_ns;
	out.frame.trial_phase = trial_phase;
	out.frame.warmup = frame_count_ < static_cast<uint64_t>(warmup_frames_);

	if (!input.grab_ok || input.mono8.empty()) {
		return out;
	}

	cv::Mat frame;
	if (use_undistort_) {
		cv::remap(input.mono8, undist_buf_, undist_map1_, undist_map2_, cv::INTER_LINEAR);
		frame = undist_buf_;
	} else {
		frame = input.mono8;
	}
	out.display_frame = frame;

	const double lr = out.frame.warmup ? 0.01 : 0.002;
	cv::Mat mask;
	bg_->apply(frame, mask, lr);
	cv::morphologyEx(mask, mask, cv::MORPH_OPEN, morph_kernel_);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	contours.erase(
		std::remove_if(contours.begin(), contours.end(), [](const auto& c) {
			const float a = static_cast<float>(cv::contourArea(c));
			return a < 200.0f || a > 60000.0f;
		}), contours.end());
	std::sort(contours.begin(), contours.end(), [](const auto& a, const auto& b) {
		return cv::contourArea(a) > cv::contourArea(b);
	});
	out.contours = contours;

	TrackState& ferret = out.frame.ferret.state;
	TrackState& prey = out.frame.prey.state;

	if (contours.size() >= 2) {
		update_track(kf_ferret_, contours[0], ferret);
		update_track(kf_prey_, contours[1], prey);
	} else if (contours.size() == 1) {
		const float area = static_cast<float>(cv::contourArea(contours[0]));
		if (area > 15000.0f) {
			update_track(kf_ferret_, contours[0], ferret);
			cv::Mat pred = kf_prey_.predict();
			prey.pos_px = {pred.at<float>(0), pred.at<float>(1)};
			prey.pos_mm = prey.pos_px * gsd_mm_px_;
			prey.speed_mm_s = std::sqrt(std::pow(pred.at<float>(2), 2)
				+ std::pow(pred.at<float>(3), 2)) * fps_ * gsd_mm_px_;
			prey.direction_deg = std::atan2(-pred.at<float>(3), pred.at<float>(2))
				* 180.0f / static_cast<float>(M_PI);
			prey.valid = true; // coast on prior velocity during occlusion
		} else {
			update_track(kf_ferret_, contours[0], ferret);
			prey.valid = false;
		}
	} else {
		ferret.valid = false;
		prey.valid = false;
	}

	fill_tracking_derived(out.frame, fps_);
	++frame_count_;
	return out;
}
