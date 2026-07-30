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
constexpr uint32_t kAxisStateClosedLoop = 8;

// Keep Set_Input_Vel alive for duration_s; returns last sample vel and axis state.
bool spin_velocity(PreyMotor& motor, float turns_s, float duration_s,
	float& vel_sample, uint32_t& last_err, uint32_t& last_state)
{
	vel_sample = 0.0f;
	last_err = 0;
	last_state = 0;
	const auto spin_end = std::chrono::steady_clock::now()
		+ std::chrono::milliseconds(static_cast<int>(duration_s * 1000.0f));
	while (std::chrono::steady_clock::now() < spin_end) {
		if (!motor.set_velocity_turns_s(turns_s)) {
			log_error("setup", "Set_Input_Vel failed during test spin");
			motor.set_velocity_turns_s(0.0f);
			return false;
		}
		vel_sample = motor.status().velocity_turns_s;
		motor.read_axis_state(last_err, last_state);
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	motor.set_velocity_turns_s(0.0f);
	return true;
}

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

	int sign = cfg.motor.chain_direction_sign;
	if (sign != -1 && sign != 1) {
		sign = 1;
	}

	// Closed-loop with err=0 but no motion usually means motor/encoder were never
	// calibrated (or marked pre_calibrated incorrectly). Offer CAN calibration first.
	if (!opts_.skip_interactive) {
		std::cout << "\nIf the motor has not been calibrated on this ODrive, run "
			"full calibration now (shaft will twitch/spin).\n";
		if (prompt_yes_no("Run ODrive full calibration over CAN? [y/n]: ")) {
			prompt_enter("Clear the chain path, then press Enter to calibrate...");
			if (!motor.run_full_calibration()) {
				log_error("setup", "ODrive full calibration failed");
				return false;
			}
		}
	}

	// Motion test always commands +1 turn/s (ODrive units). Direction sign is
	// only applied later in the short jog / saved chain_direction_sign.
	const float cmd_turns_s = kTestTurnsPerS;

	// Arm closed-loop only after the operator is ready — a prior Enter wait
	// let the ODrive watchdog disarm before Set_Input_Vel started.
	if (!opts_.skip_interactive) {
		std::cout << "\nTest spin: " << cmd_turns_s << " turns/s for "
			<< kTestDurationS << "s\nKeep hands clear of the chain.\n";
		prompt_enter("Press Enter to arm motor and start test spin...");
	}

	if (!motor.enter_velocity_mode()) {
		log_error("setup", "Failed to enter closed-loop velocity mode over CAN");
		return false;
	}

	const float pos_before = motor.read_position_turns();
	float vel_sample = 0.0f;
	uint32_t axis_err = 0;
	uint32_t axis_state = 0;
	if (!spin_velocity(motor, cmd_turns_s, kTestDurationS, vel_sample, axis_err, axis_state)) {
		return false;
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(200));
	const float pos_after = motor.read_position_turns();
	const float delta_turns = pos_after - pos_before;

	std::cout << "Encoder delta: " << delta_turns << " motor turns"
		<< " (sample vel " << vel_sample << " turns/s"
		<< ", axis_state=" << axis_state
		<< ", err=0x" << std::hex << axis_err << std::dec << ")\n";
	if (axis_state != kAxisStateClosedLoop) {
		log_error("setup",
			"Axis left closed-loop during spin (watchdog/disarm) — check enable_watchdog timeout");
		return false;
	}
	// ~2 turns expected at 1 turn/s × 2s; sub-turn noise is not real motion.
	if (std::fabs(delta_turns) < 0.1f) {
		log_error("setup",
			"Encoder barely moved — motor/chain did not spin. Check ODrive GUI "
			"calibration, vel_limit, coupling to the chain, then retry");
		return false;
	}

	float measured_mm = 100.0f;
	if (!opts_.skip_interactive) {
		if (!prompt_yes_no("Did you see the chain move during the test spin? [y/n]: ")) {
			log_error("setup",
				"No chain motion — fix mechanics/ODrive velocity control before measuring mm");
			return false;
		}
		measured_mm = prompt_float(
			"Measure chain travel during the test (mm, absolute value, e.g. 100): ");
	}

	cfg.motor.chain_mm_per_motor_turn = std::fabs(measured_mm / delta_turns);
	cfg.motor.pulley_radius_m = cfg.motor.chain_mm_per_motor_turn / (kTwoPi * 1000.0f);

	std::cout << "chain_mm_per_motor_turn = " << cfg.motor.chain_mm_per_motor_turn << '\n';

	if (!opts_.skip_interactive) {
		std::cout << "Short direction jog (0.5 s)...\n";
		// Re-arm: measurement prompt can outlast the watchdog.
		if (!motor.enter_velocity_mode()) {
			log_error("setup", "Failed to re-enter closed-loop for direction jog");
			return false;
		}
		float jog_vel = 0.0f;
		uint32_t jog_err = 0;
		uint32_t jog_state = 0;
		if (!spin_velocity(motor, 0.5f * static_cast<float>(sign), 0.5f,
				jog_vel, jog_err, jog_state)) {
			return false;
		}
		cfg.motor.chain_direction_sign = prompt_yes_no(
			"Did the chain move in the prey-flee direction? [y/n]: ") ? 1 : -1;
	}

	if (!save_motor_calibration(config_path, cfg.motor)) {
		return false;
	}
	log_info("setup", "ODrive chain calibration saved");
	return true;
}
