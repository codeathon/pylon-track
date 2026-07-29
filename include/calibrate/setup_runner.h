#pragma once

#include "calibrate/setup_options.h"

// Bundled one-time rig setup orchestrator.
class SetupRunner {
public:
	explicit SetupRunner(SetupOptions opts);

	int run();

private:
	SetupOptions opts_;

	bool should_run(SetupStep step) const;
};
