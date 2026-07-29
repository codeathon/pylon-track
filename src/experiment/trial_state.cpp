#include "experiment/trial_state.h"

const char* trial_phase_name(TrialPhase phase) {
	switch (phase) {
	case TrialPhase::Idle: return "idle";
	case TrialPhase::Warmup: return "warmup";
	case TrialPhase::Armed: return "armed";
	case TrialPhase::Running: return "running";
	case TrialPhase::Ended: return "ended";
	}
	return "unknown";
}

void TrialStateMachine::begin_warmup() {
	phase_ = TrialPhase::Warmup;
}

void TrialStateMachine::on_frame(uint64_t frame_index, int warmup_frames) {
	if (phase_ == TrialPhase::Warmup
		&& frame_index >= static_cast<uint64_t>(warmup_frames)) {
		phase_ = TrialPhase::Armed;
	}
}

bool TrialStateMachine::start_trial() {
	if (phase_ != TrialPhase::Armed) {
		return false;
	}
	phase_ = TrialPhase::Running;
	return true;
}

bool TrialStateMachine::end_trial() {
	if (phase_ != TrialPhase::Running) {
		return false;
	}
	phase_ = TrialPhase::Ended;
	return true;
}

bool TrialStateMachine::reset_trial() {
	if (phase_ != TrialPhase::Ended) {
		return false;
	}
	phase_ = TrialPhase::Armed;
	return true;
}
