#include "tracker/ferret_tracker.h"
#include "log/logger.h"
#include "vision/camera_frame.h"

#include <chrono>
#include <sstream>

FerretTracker::FerretTracker(bool enable_display, int warmup_frames, float gsd_mm_px,
	std::optional<CameraCalib> calib)
	: enable_display_(enable_display)
	, warmup_frames_(warmup_frames)
	, pipeline_(warmup_frames, gsd_mm_px, FPS, calib)
{
}

void FerretTracker::set_experiment_hooks(TrialStateMachine* fsm,
	SessionRecorder* recorder)
{
	trial_fsm_ = fsm;
	session_recorder_ = recorder;
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

bool FerretTracker::get_tracking_frame(TrackingFrame& out) {
	std::lock_guard<std::mutex> lock(tracking_mutex_);
	if (!tracking_ready_) {
		return false;
	}
	out = latest_tracking_;
	return true;
}

void FerretTracker::publish_display_snapshot(const cv::Mat& frame,
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

void FerretTracker::OnImageGrabbed(Pylon::CInstantCamera&,
	const Pylon::CGrabResultPtr& result)
{
	const auto t0 = std::chrono::steady_clock::now();
	const int64_t host_time_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count();

	const CameraFrame input = make_camera_frame(result, frame_count_, host_time_ns);

	TrialPhase phase = TrialPhase::Warmup;
	if (trial_fsm_) {
		trial_fsm_->on_frame(frame_count_, warmup_frames_);
		phase = trial_fsm_->phase();
	} else if (frame_count_ >= static_cast<uint64_t>(warmup_frames_)) {
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

	if (enable_display_ && !out.display_frame.empty()) {
		publish_display_snapshot(out.display_frame, out.contours, out.frame.warmup);
	}

	// Log frame processing time ~1 Hz at 200 fps (Debug only).
	if (frame_count_ % 200 == 0) {
		const auto dt_us = std::chrono::duration_cast<std::chrono::microseconds>(
			std::chrono::steady_clock::now() - t0).count();
		std::ostringstream oss;
		oss << "OnImageGrabbed " << dt_us << " us";
		log_debug("tracker", oss.str());

		// Log speed and distance ~1 Hz when both tracks are valid.
		if (ferret.valid && prey.valid) {
			float dx = ferret.pos_mm.x - prey.pos_mm.x;
			float dy = ferret.pos_mm.y - prey.pos_mm.y;
			float dist_mm = std::sqrt(dx * dx + dy * dy);
			std::ostringstream koss;
			koss << "ferret_speed=" << static_cast<int>(ferret.speed_mm_s)
			     << "mm/s  prey_speed=" << static_cast<int>(prey.speed_mm_s)
			     << "mm/s  distance=" << static_cast<int>(dist_mm) << "mm";
			log_info("tracker", koss.str());
		}
	}
}
