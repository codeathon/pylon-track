#pragma once

#include <cstdint>
#include <string>

// Experiment lifecycle — operator manual start/end in v1.
enum class TrialPhase : uint8_t {
	Idle,
	Warmup,
	Armed,
	Running,
	Ended
};

const char* trial_phase_name(TrialPhase phase);

class TrialStateMachine {
public:
	void begin_warmup();
	// Call each processed frame; auto Warmup → Armed after warmup_frames.
	void on_frame(uint64_t frame_index, int warmup_frames);

	bool start_trial();
	bool end_trial();
	bool reset_trial();

	TrialPhase phase() const { return phase_; }

private:
	TrialPhase phase_ = TrialPhase::Idle;
};
