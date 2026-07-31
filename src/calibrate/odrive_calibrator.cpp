#include "calibrate/odrive_calibrator.h"

#include <atomic>
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
constexpr float kTwoPi = 6.283185307f;
constexpr uint32_t kAxisStateClosedLoop = 8;

// Keep Set_Input_Vel alive for duration_s; returns last sample vel and axis
// state. Checks `running` (may be null) every 100ms so Ctrl-C stops the
// motor immediately instead of riding out the full spin duration.
bool spin_velocity(PreyMotor& motor, float turns_s, float duration_s,
	float& vel_sample, uint32_t& last_err, uint32_t& last_state,
	std::atomic<bool>* running)
{
	vel_sample = 0.0f;
	last_err = 0;
	last_state = 0;
	const auto spin_end = std::chrono::steady_clock::now()
		+ std::chrono::milliseconds(static_cast<int>(duration_s * 1000.0f));
	while (std::chrono::steady_clock::now() < spin_end) {
		if (running && !running->load()) {
			log_error("setup", "Test spin interrupted — motor stopped");
			motor.set_velocity_turns_s(0.0f);
			return false;
		}
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
	// Persist the sanitized value now so --skip-interactive runs never write
	// back the original invalid sign; the interactive jog below can still
	// refine it from the operator's confirmation.
	cfg.motor.chain_direction_sign = sign;

	// Closed-loop with err=0 but no motion usually means motor/encoder were never
	// calibrated (or marked pre_calibrated incorrectly). Offer CAN calibration first.
	if (!opts_.skip_interactive) {
		std::cout << "\nIf the motor has not been calibrated on this ODrive, run "
			"full calibration now (shaft will twitch/spin).\n";
		const std::optional<bool> run_calib =
			prompt_yes_no("Run ODrive full calibration over CAN? [y/n]: ");
		if (!run_calib) {
			log_error("setup", "No input received (stdin closed) — aborting calibration");
			return false;
		}
		if (*run_calib) {
			if (!prompt_enter("Clear the chain path, then press Enter to calibrate...")) {
				log_error("setup", "No input received (stdin closed) — aborting calibration");
				return false;
			}
			if (!motor.run_full_calibration()) {
				log_error("setup", "ODrive full calibration failed");
				return false;
			}
		}
	}

	// Motion test always commands +1 turn/s (ODrive units). Direction sign is
	// only applied later in the short jog / saved chain_direction_sign.
	const float cmd_turns_s = kTestTurnsPerS;
	const float spin_s = (opts_.spin_seconds > 0.5f) ? opts_.spin_seconds : 0.5f;

	// Arm closed-loop only after the operator is ready — a prior Enter wait
	// let the ODrive watchdog disarm before Set_Input_Vel started.
	if (!opts_.skip_interactive) {
		std::cout << "\nTest spin: " << cmd_turns_s << " turns/s for "
			<< spin_s << "s\nKeep hands clear of the chain.\n";
		if (!prompt_enter("Press Enter to arm motor and start test spin...")) {
			log_error("setup", "No input received (stdin closed) — aborting calibration");
			return false;
		}
	}

	if (!motor.enter_velocity_mode()) {
		log_error("setup", "Failed to enter closed-loop velocity mode over CAN");
		return false;
	}

	const float pos_before = motor.read_position_turns();
	float vel_sample = 0.0f;
	uint32_t axis_err = 0;
	uint32_t axis_state = 0;
	if (!spin_velocity(motor, cmd_turns_s, spin_s, vel_sample, axis_err, axis_state,
			opts_.running)) {
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
	// Expect ~spin_s turns at 1 turn/s; sub-turn noise is not real motion.
	if (std::fabs(delta_turns) < 0.1f) {
		log_error("setup",
			"Encoder barely moved — motor/chain did not spin. Check ODrive GUI "
			"calibration, vel_limit, coupling to the chain, then retry");
		return false;
	}

	float measured_mm = 100.0f;
	if (!opts_.skip_interactive) {
		const std::optional<bool> saw_motion =
			prompt_yes_no("Did you see the chain move during the test spin? [y/n]: ");
		if (!saw_motion) {
			log_error("setup", "No input received (stdin closed) — aborting calibration");
			return false;
		}
		if (!*saw_motion) {
			log_error("setup",
				"No chain motion — fix mechanics/ODrive velocity control before measuring mm");
			return false;
		}
		const std::optional<float> answer = prompt_float(
			"Measure chain travel during the test (mm, absolute value, e.g. 100): ");
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
		// Re-arm: measurement prompt can outlast the watchdog.
		if (!motor.enter_velocity_mode()) {
			log_error("setup", "Failed to re-enter closed-loop for direction jog");
			return false;
		}
		float jog_vel = 0.0f;
		uint32_t jog_err = 0;
		uint32_t jog_state = 0;
		if (!spin_velocity(motor, 0.5f * static_cast<float>(sign), 0.5f,
				jog_vel, jog_err, jog_state, opts_.running)) {
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
