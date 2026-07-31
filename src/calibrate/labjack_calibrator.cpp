#include "calibrate/labjack_calibrator.h"

#include <chrono>
#include <optional>
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
	const bool completed = interruptible_sleep_ms(opts_, 3000);
	shuttle.stop();
	if (!completed) {
		log_error("setup", "Shuttle test interrupted — motor stopped");
		return false;
	}

	if (!opts_.skip_interactive) {
		const std::optional<bool> ok = prompt_yes_no("Did the motor wobble back and forth? [y/n]: ");
		if (!ok) {
			log_error("setup", "No input received (stdin closed) — aborting");
			return false;
		}
		if (!*ok) {
			log_error("setup", "Operator rejected shuttle motor motion");
			return false;
		}
	}
	log_info("setup", "LabJack shuttle motor verified");
	return true;
}
