#include "tracker/ferret_tracker.h"

FerretTracker::FerretTracker(bool enable_display, int warmup_frames, float gsd_mm_px,
	std::optional<CameraCalib> calib, std::optional<ArenaMaskConfig> mask,
	std::optional<VisionConfig> vision)
	: CameraTrackingService({warmup_frames, gsd_mm_px, FPS, calib, mask, vision,
		enable_display})
{
}

void FerretTracker::set_experiment_hooks(TrialStateMachine* fsm,
	SessionRecorder* recorder)
{
	set_trial_fsm(fsm);
	set_session_recorder(recorder);
}
