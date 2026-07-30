#include "calibrate/odrive_calibrator.h"

#include <chrono>
#include <cmath>
#include <iostream>
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
		log_error("setup", can_interface_down_hint(cfg.motor.can_interface));
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

	if (!opts_.skip_interactive) {
		std::cout << "\nTest spin: " << (kTestTurnsPerS * sign) << " turns/s for "
			<< kTestDurationS << "s\nKeep hands clear of the chain.\n";
		prompt_enter("Press Enter to start test spin...");
	}

	const float pos_before = motor.read_position_turns();
	const float cmd_turns_s = kTestTurnsPerS * static_cast<float>(sign);
	// Re-send velocity while spinning — ODrive watchdog disarms if Set_Input_Vel
	// stops (a single command + 2s sleep often yields ~zero encoder delta).
	const auto spin_end = std::chrono::steady_clock::now()
		+ std::chrono::milliseconds(static_cast<int>(kTestDurationS * 1000.0f));
	float vel_sample = 0.0f;
	while (std::chrono::steady_clock::now() < spin_end) {
		if (!motor.set_velocity_turns_s(cmd_turns_s)) {
			log_error("setup", "Set_Input_Vel failed during test spin");
			motor.set_velocity_turns_s(0.0f);
			return false;
		}
		vel_sample = motor.status().velocity_turns_s;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	motor.set_velocity_turns_s(0.0f);
	std::this_thread::sleep_for(std::chrono::milliseconds(200));
	const float pos_after = motor.read_position_turns();
	const float delta_turns = pos_after - pos_before;

	std::cout << "Encoder delta: " << delta_turns << " motor turns"
		<< " (sample vel " << vel_sample << " turns/s)\n";
	if (std::fabs(delta_turns) < 1e-3f) {
		log_error("setup",
			"Encoder did not move — confirm motor spun; if not, run motor+encoder "
			"calibration in ODrive GUI, check enable/limits, then retry");
		return false;
	}

	float measured_mm = 100.0f;
	if (!opts_.skip_interactive) {
		measured_mm = prompt_float(
			"Measure chain travel during the test (mm, absolute value): ");
	}

	cfg.motor.chain_mm_per_motor_turn = std::fabs(measured_mm / delta_turns);
	cfg.motor.pulley_radius_m = cfg.motor.chain_mm_per_motor_turn / (kTwoPi * 1000.0f);

	std::cout << "chain_mm_per_motor_turn = " << cfg.motor.chain_mm_per_motor_turn << '\n';

	if (!opts_.skip_interactive) {
		std::cout << "Short direction jog (0.5 s)...\n";
		motor.set_velocity_turns_s(0.5f * static_cast<float>(sign));
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		motor.set_velocity_turns_s(0.0f);
		cfg.motor.chain_direction_sign = prompt_yes_no(
			"Did the chain move in the prey-flee direction? [y/n]: ") ? 1 : -1;
	}

	if (!save_motor_calibration(config_path, cfg.motor)) {
		return false;
	}
	log_info("setup", "ODrive chain calibration saved");
	return true;
}
