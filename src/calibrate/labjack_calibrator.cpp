#include "calibrate/labjack_calibrator.h"

#include <chrono>
#include <thread>

#include "calibrate/setup_util.h"
#include "log/logger.h"
#include "motor/shuttle_motor.h"

LabjackCalibrator::LabjackCalibrator(const SetupOptions& opts) : opts_(opts) {}

bool LabjackCalibrator::run(const ArenaExperimentConfig& cfg) {
	if (cfg.shuttle.backend == "noop") {
		log_info("setup", "Shuttle motor noop backend — skipping hardware test");
		return true;
	}

	ShuttleMotor shuttle(cfg.shuttle);
	if (!shuttle.connect()) {
		log_error("setup", "Shuttle motor connect failed");
		return false;
	}

	log_info("setup", "Wobbling shuttle motor for 3s...");
	shuttle.start();
	std::this_thread::sleep_for(std::chrono::seconds(3));
	shuttle.stop();

	if (!opts_.skip_interactive) {
		const bool ok = prompt_yes_no("Did the motor wobble back and forth? [y/n]: ");
		if (!ok) {
			log_error("setup", "Operator rejected shuttle motor motion");
			return false;
		}
	}
	log_info("setup", "LabJack shuttle motor verified");
	return true;
}
