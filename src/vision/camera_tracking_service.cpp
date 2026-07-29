#include "vision/camera_tracking_service.h"
#include "log/logger.h"
#include "vision/camera_frame.h"

#include <chrono>
#include <cmath>
#include <sstream>

CameraTrackingService::CameraTrackingService(const Options& opts)
	: opts_(opts)
	, pipeline_(opts.warmup_frames, opts.gsd_mm_px, opts.fps,
		opts.calib, opts.mask, opts.vision)
{
}

bool CameraTrackingService::get_display_snapshot(DisplaySnapshot& out) {
	if (!opts_.enable_display) {
		return false;
	}
	std::lock_guard<std::mutex> lock(display_mutex_);
	if (!display_ready_) {
		return false;
	}
	out = display_snapshot_;
	return true;
}

bool CameraTrackingService::get_tracking_frame(TrackingFrame& out) {
	std::lock_guard<std::mutex> lock(tracking_mutex_);
	if (!tracking_ready_) {
		return false;
	}
	out = latest_tracking_;
	return true;
}

void CameraTrackingService::publish_display_snapshot(const cv::Mat& frame,
	const std::vector<std::vector<cv::Point>>& contours,
	bool warmup)
{
	std::lock_guard<std::mutex> lock(display_mutex_);
	display_snapshot_.frame = frame.clone();
	display_snapshot_.contours = contours;
	display_snapshot_.ferret = ferret;
	display_snapshot_.prey = prey;
	display_snapshot_.frame_count = frame_count_;
	display_snapshot_.warmup = warmup;
	display_ready_ = true;
}

void CameraTrackingService::OnImageGrabbed(Pylon::CInstantCamera&,
	const Pylon::CGrabResultPtr& result)
{
	const auto t0 = std::chrono::steady_clock::now();
	const int64_t host_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count();

	const CameraFrame input = make_camera_frame(result, frame_count_, host_time_ns);

	TrialPhase phase = TrialPhase::Warmup;
	if (trial_fsm_) {
		trial_fsm_->on_frame(frame_count_, opts_.warmup_frames);
		phase = trial_fsm_->phase();
	} else if (frame_count_ >= static_cast<uint64_t>(opts_.warmup_frames)) {
		phase = TrialPhase::Armed;
	}

	const TrackingProcessOutput out = pipeline_.process(input, phase);
	frame_count_ = pipeline_.frame_count();

	ferret = out.frame.ferret.state;
	prey = out.frame.prey.state;

	{
		std::lock_guard<std::mutex> lock(tracking_mutex_);
		latest_tracking_ = out.frame;
		tracking_ready_ = true;
	}

	if (session_recorder_ && session_recorder_->is_open()) {
		session_recorder_->log_frame(out.frame);
	}

	if (opts_.enable_display && !out.display_frame.empty()) {
		publish_display_snapshot(out.display_frame, out.contours, out.frame.warmup);
	}

	if (frame_count_ % 200 == 0) {
		const auto dt_us = std::chrono::duration_cast<std::chrono::microseconds>(
			std::chrono::steady_clock::now() - t0).count();
		std::ostringstream oss;
		oss << "OnImageGrabbed " << dt_us << " us";
		log_debug("tracker", oss.str());

		if (ferret.valid && prey.valid) {
			const float dx = ferret.pos_mm.x - prey.pos_mm.x;
			const float dy = ferret.pos_mm.y - prey.pos_mm.y;
			const float dist_mm = std::sqrt(dx * dx + dy * dy);
			std::ostringstream koss;
			koss << "ferret_speed=" << static_cast<int>(ferret.speed_mm_s)
			     << "mm/s  prey_speed=" << static_cast<int>(prey.speed_mm_s)
			     << "mm/s  distance=" << static_cast<int>(dist_mm) << "mm";
			log_info("tracker", koss.str());
		}
	}
}
