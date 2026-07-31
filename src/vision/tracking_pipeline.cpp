#include "vision/tracking_pipeline.h"

#include <algorithm>
#include <cmath>
#include "log/logger.h"
#include "tracker/tracker.h"

namespace {
// ~0.15s at 200fps — long enough to ride out a pulley/ignore-region
// occlusion, short enough that a genuinely-gone animal stops being
// reported as "valid" instead of coasting on a stale prediction forever.
constexpr int kMaxCoastFrames = 30;
} // namespace

TrackingPipeline::TrackingPipeline(int warmup_frames, float gsd_mm_px, float fps,
	std::optional<CameraCalib> calib, std::optional<ArenaMaskConfig> mask_cfg,
	std::optional<VisionConfig> vision_cfg)
	: warmup_frames_(warmup_frames)
	, gsd_mm_px_(gsd_mm_px)
	, fps_(fps)
	, bg_(cv::createBackgroundSubtractorMOG2(500, 16, false))
	, kf_ferret_(make_kalman(fps))
	, kf_prey_(make_kalman(fps))
	, morph_kernel_(cv::getStructuringElement(cv::MORPH_ELLIPSE, {7, 7}))
	, arena_mask_(mask_cfg.value_or(ArenaMaskConfig{}))
	, associator_(vision_cfg.value_or(VisionConfig{}))
{
	if (calib && calib->enabled()) {
		use_undistort_ = true;
		undist_map1_ = calib->map1;
		undist_map2_ = calib->map2;
		undist_buf_ = cv::Mat(calib->image_size, CV_8UC1);
		log_info("tracker", "Lens undistort enabled");
	}
	if (arena_mask_.enabled()) {
		log_info("tracker", "Arena mask enabled (ignore_regions / track_roi)");
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

void TrackingPipeline::coast_track(cv::KalmanFilter& kf, const TrackState& prior,
	TrackState& state, int miss_streak, int max_miss_frames)
{
	if (!prior.valid || miss_streak > max_miss_frames) {
		return; // stays invalid — nothing plausible to coast from
	}
	cv::Mat pred = kf.predict();
	const float vx_px = pred.at<float>(2);
	const float vy_px = pred.at<float>(3);
	state.pos_px = {pred.at<float>(0), pred.at<float>(1)};
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
	bg_->apply(frame, mask_, lr);
	cv::morphologyEx(mask_, mask_, cv::MORPH_OPEN, morph_kernel_);
	arena_mask_.apply(mask_);

	std::vector<std::vector<cv::Point>> contours;
	cv::findContours(mask_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
	contours.erase(
		std::remove_if(contours.begin(), contours.end(), [](const auto& c) {
			const float a = static_cast<float>(cv::contourArea(c));
			return a < 200.0f || a > 60000.0f;
		}), contours.end());
	out.contours = contours;

	TrackState& ferret = out.frame.ferret.state;
	TrackState& prey = out.frame.prey.state;
	ferret.valid = false;
	prey.valid = false;

	const AssociationResult assoc = associator_.associate(
		contours, ferret_prior_, prey_prior_);
	out.frame.quality.ferret_confidence = assoc.ferret_confidence;
	out.frame.quality.prey_confidence = assoc.prey_confidence;
	out.frame.quality.reject_reason = assoc.reject_reason;

	// Every frame with no matching detection still calls predict() (via
	// coast_track) so the filter's internal state keeps advancing with real
	// time. Without this, a multi-frame gap (occlusion, a rejected/noisy
	// contour, a merge) left the filter frozen — the next successful
	// correct() then modeled only 1/fps of motion against a measurement
	// that actually moved several frames' worth of distance, producing a
	// large bogus speed/direction spike on reacquisition.
	if (assoc.ferret_idx >= 0) {
		update_track(kf_ferret_, contours[static_cast<size_t>(assoc.ferret_idx)], ferret);
		ferret_miss_streak_ = 0;
	} else {
		++ferret_miss_streak_;
		coast_track(kf_ferret_, ferret_prior_, ferret, ferret_miss_streak_, kMaxCoastFrames);
		if (ferret.valid) {
			out.frame.quality.ferret_confidence *= 0.5f;
		}
	}

	if (assoc.prey_idx >= 0) {
		update_track(kf_prey_, contours[static_cast<size_t>(assoc.prey_idx)], prey);
		prey_miss_streak_ = 0;
	} else {
		++prey_miss_streak_;
		coast_track(kf_prey_, prey_prior_, prey, prey_miss_streak_, kMaxCoastFrames);
		if (prey.valid) {
			out.frame.quality.prey_confidence *= 0.5f;
		}
	}

	ferret_prior_ = ferret;
	prey_prior_ = prey;

	fill_tracking_derived(out.frame, fps_);
	++frame_count_;
	return out;
}
