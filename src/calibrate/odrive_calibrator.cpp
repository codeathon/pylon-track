#include "calibrate/odrive_calibrator.h"

#include <chrono>
#include <cmath>
#include <iostream>
#include <optional>
#include <thread>

#include "calibrate/setup_util.h"
#include "log/logger.h"
#include "motor/motor_config.h"
#include "motor/prey_motor.h"

namespace {

constexpr float kTestTurnsPerS = 1.0f;
constexpr float kTestDurationS = 2.0f;
constexpr float kTwoPi = 6.283185307f;

} // namespace

ODriveCalibrator::ODriveCalibrator(const SetupOptions& opts) : opts_(opts) {}

bool ODriveCalibrator::run(const std::string& config_path, ArenaExperimentConfig& cfg) {
	if (!can_interface_up(cfg.motor.can_interface)) {
		log_error("setup", "CAN interface " + cfg.motor.can_interface
			+ " is not up — run: sudo ip link set "
			+ cfg.motor.can_interface + " up type can bitrate 250000");
		return false;
	}

	PreyMotor motor(prey_motor_from_config(cfg.motor));
	if (!motor.connect()) {
		log_error("setup", "Prey motor connect failed");
		return false;
	}
	if (!motor.status().heartbeat_ok) {
		log_error("setup", "Prey motor heartbeat failed");
		return false;
	}
	if (!motor.enter_velocity_mode()) {
		log_error("setup", "Failed to enter closed-loop velocity mode over CAN");
		return false;
	}

	int sign = cfg.motor.chain_direction_sign;
	if (sign != -1 && sign != 1) {
		sign = 1;
	}
	// Persist the sanitized value now so --skip-interactive runs never write
	// back the original invalid sign; the interactive jog below can still
	// refine it from the operator's confirmation.
	cfg.motor.chain_direction_sign = sign;

	if (!opts_.skip_interactive) {
		std::cout << "\nTest spin: " << (kTestTurnsPerS * sign) << " turns/s for "
			<< kTestDurationS << "s\nKeep hands clear of the chain.\n";
		prompt_enter("Press Enter to start test spin...");
	}

	const float pos_before = motor.read_position_turns();
	motor.set_velocity_turns_s(kTestTurnsPerS * static_cast<float>(sign));
	const bool spin_completed = interruptible_sleep_ms(opts_,
		static_cast<int>(kTestDurationS * 1000.0f));
	motor.set_velocity_turns_s(0.0f);
	if (!spin_completed) {
		log_error("setup", "Test spin interrupted — motor stopped");
		return false;
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(200));
	const float pos_after = motor.read_position_turns();
	const float delta_turns = pos_after - pos_before;

	std::cout << "Encoder delta: " << delta_turns << " motor turns\n";
	if (std::fabs(delta_turns) < 1e-4f) {
		log_error("setup", "Encoder did not move — check wiring and ODrive calibration");
		return false;
	}

	float measured_mm = 100.0f;
	if (!opts_.skip_interactive) {
		const std::optional<float> answer = prompt_float(
			"Measure chain travel during the test (mm, absolute value): ");
		if (!answer) {
			log_error("setup", "No input received (stdin closed) — aborting calibration");
			return false;
		}
		measured_mm = *answer;
	}

	cfg.motor.chain_mm_per_motor_turn = std::fabs(measured_mm / delta_turns);
	cfg.motor.pulley_radius_m = cfg.motor.chain_mm_per_motor_turn / (kTwoPi * 1000.0f);

	std::cout << "chain_mm_per_motor_turn = " << cfg.motor.chain_mm_per_motor_turn << '\n';

	if (!opts_.skip_interactive) {
		std::cout << "Short direction jog (0.5 s)...\n";
		motor.set_velocity_turns_s(0.5f * static_cast<float>(sign));
		const bool jog_completed = interruptible_sleep_ms(opts_, 500);
		motor.set_velocity_turns_s(0.0f);
		if (!jog_completed) {
			log_error("setup", "Direction jog interrupted — motor stopped");
			return false;
		}
		const std::optional<bool> confirmed = prompt_yes_no(
			"Did the chain move in the prey-flee direction? [y/n]: ");
		if (!confirmed) {
			log_error("setup", "No input received (stdin closed) — aborting calibration");
			return false;
		}
		cfg.motor.chain_direction_sign = *confirmed ? 1 : -1;
	}

	if (!save_motor_calibration(config_path, cfg.motor)) {
		return false;
	}
	log_info("setup", "ODrive chain calibration saved");
	return true;
}
