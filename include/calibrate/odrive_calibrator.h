#pragma once

#include "calibrate/setup_options.h"
#include "experiment/arena_config.h"

// ODrive chain calibration over CAN — writes motor section to arena_experiment.json.
class ODriveCalibrator {
public:
	explicit ODriveCalibrator(const SetupOptions& opts);

	bool run(const std::string& config_path, ArenaExperimentConfig& cfg);

private:
	SetupOptions opts_;
};
