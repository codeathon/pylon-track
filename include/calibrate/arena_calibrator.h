#pragma once

#include "calibrate/setup_options.h"
#include "experiment/arena_config.h"

// Interactive arena mask editor — writes vision section to arena_experiment.json.
class ArenaCalibrator {
public:
	explicit ArenaCalibrator(const SetupOptions& opts);

	bool run(const std::string& config_path, ArenaExperimentConfig& cfg);

private:
	SetupOptions opts_;
};
