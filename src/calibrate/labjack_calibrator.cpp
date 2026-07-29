#include "calibrate/labjack_calibrator.h"

#include <iostream>

#include "calibrate/setup_util.h"
#include "log/logger.h"
#include "motor/trap_door_motor.h"

LabjackCalibrator::LabjackCalibrator(const SetupOptions& opts) : opts_(opts) {}

bool LabjackCalibrator::run(const ArenaExperimentConfig& cfg) {
	if (cfg.trap_door.backend == "noop") {
		log_info("setup", "Trap door noop backend — skipping hardware test");
		return true;
	}

	TrapDoorMotor trap(cfg.trap_door);
	if (!trap.connect()) {
		log_error("setup", "Trap door connect failed");
		return false;
	}

	log_info("setup", "Opening trap door...");
	if (!trap.open_trap()) {
		log_error("setup", "Trap door open failed");
		return false;
	}

	if (!opts_.skip_interactive) {
		const bool ok = prompt_yes_no("Did the trap door open and close? [y/n]: ");
		if (!ok) {
			log_error("setup", "Operator rejected trap door motion");
			return false;
		}
	}
	log_info("setup", "LabJack trap door verified");
	return true;
}
