#pragma once

#include "calibrate/setup_options.h"
#include "experiment/arena_config.h"

// LabJack shuttle motor connect + wobble verification.
class LabjackCalibrator {
public:
	explicit LabjackCalibrator(const SetupOptions& opts);

	bool run(const ArenaExperimentConfig& cfg);

private:
	SetupOptions opts_;
};
