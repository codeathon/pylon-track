#include "motor/prey_motor.h"

#include <cmath>
#include "log/logger.h"

namespace {

constexpr float kTwoPi = 6.283185307f;

} // namespace

PreyMotor::PreyMotor(const PreyMotorConfig& cfg)
	: cfg_(cfg)
	, can_({cfg.can_interface, cfg.node_id})
{
	if (cfg.chain_mm_per_motor_turn > 0.0f) {
		mm_per_turn_ = cfg.chain_mm_per_motor_turn;
	} else {
		mm_per_turn_ = kTwoPi * cfg.pulley_radius_m * 1000.0f;
	}
}

bool PreyMotor::connect() {
	if (!can_.open()) {
		status_.connected = false;
		return false;
	}
	status_.connected = true;
	// Autobaud drives stay silent until they see host traffic — beacon first.
	status_.heartbeat_ok = can_.wake_autobaud(3000);
	refresh_status();
	log_info("motor", status_.heartbeat_ok
		? "PreyMotor connected with heartbeat"
		: "PreyMotor connected (no heartbeat yet)");
	return true;
}

float PreyMotor::chain_mps_to_turns_s(float chain_mps) const {
	if (mm_per_turn_ <= 0.0f) {
		return 0.0f;
	}
	const float chain_mm_s = chain_mps * 1000.0f;
	const float turns_s = chain_mm_s / mm_per_turn_;
	return turns_s * static_cast<float>(cfg_.chain_direction_sign);
}

float PreyMotor::turns_s_to_chain_mps(float turns_s) const {
	if (mm_per_turn_ <= 0.0f) {
		return 0.0f;
	}
	const float chain_mm_s = turns_s * mm_per_turn_;
	return (chain_mm_s / 1000.0f) * static_cast<float>(cfg_.chain_direction_sign);
}

float PreyMotor::turns_to_chain_mm(float turns) const {
	return turns * mm_per_turn_ * static_cast<float>(cfg_.chain_direction_sign);
}

float PreyMotor::chain_mm_to_turns(float chain_mm) const {
	if (mm_per_turn_ <= 0.0f) {
		return 0.0f;
	}
	return (chain_mm / mm_per_turn_) * static_cast<float>(cfg_.chain_direction_sign);
}

void PreyMotor::refresh_status() const {
	if (!status_.connected) {
		return;
	}
	// CAN round-trip happens outside the lock (ODriveCan serializes its own
	// I/O) — only the status_ write itself needs to be atomic w.r.t. other
	// threads calling status()/read_position_turns() concurrently.
	float pos = 0.0f;
	float vel = 0.0f;
	if (can_.get_encoder_estimates(pos, vel)) {
		std::lock_guard<std::mutex> lock(status_mutex_);
		status_.position_turns = pos;
		status_.velocity_turns_s = vel;
		status_.chain_position_mm = turns_to_chain_mm(pos);
		status_.chain_velocity_mps = turns_s_to_chain_mps(vel);
	}
}

void PreyMotor::apply(const MotorCommand& cmd) {
	if (!status_.connected) {
		return;
	}
	if (cmd.estop || cmd.mode == MotorMode::Estop) {
		estop();
		return;
	}
	if (cmd.mode == MotorMode::Idle) {
		stop();
		return;
	}
	if (cmd.mode == MotorMode::Velocity) {
		if (!has_valid_chain_scale() && std::fabs(cmd.velocity_mps) > 1e-6f) {
			// Why: both chain_mm_per_motor_turn and pulley_radius_m were 0 in lab JSON.
			log_error("motor",
				"Cannot command chain velocity — set motor.chain_mm_per_motor_turn "
				"(e.g. 157) or motor.pulley_radius_m in arena_experiment.json");
			return;
		}
		const float turns_s = chain_mps_to_turns_s(cmd.velocity_mps);
		// No refresh_status here — MotionPlanner runs ~50 Hz and must keep
		// Set_Input_Vel flowing faster than the ODrive watchdog.
		if (!can_.set_input_velocity(turns_s, 0.0f)) {
			log_error("motor", "Set_Input_Vel failed ("
				+ std::to_string(turns_s) + " turns/s)");
		}
	}
}

void PreyMotor::stop() {
	if (!status_.connected) {
		return;
	}
	can_.set_input_velocity(0.0f, 0.0f);
	refresh_status();
}

void PreyMotor::estop() {
	if (!status_.connected) {
		return;
	}
	can_.send_estop();
	can_.set_input_velocity(0.0f, 0.0f);
	refresh_status();
}

MotorStatus PreyMotor::status() const {
	refresh_status();
	std::lock_guard<std::mutex> lock(status_mutex_);
	return status_;
}

bool PreyMotor::is_connected() const {
	// No CAN — tick() calls this every ~20 ms and must stay non-blocking.
	std::lock_guard<std::mutex> lock(status_mutex_);
	return status_.connected;
}

bool PreyMotor::try_sample_encoder(float& pos_turns, float& vel_turns_s,
	int timeout_ms)
{
	if (!is_connected()) {
		return false;
	}
	float pos = 0.0f;
	float vel = 0.0f;
	if (!can_.get_encoder_estimates(pos, vel, timeout_ms)) {
		return false;
	}
	std::lock_guard<std::mutex> lock(status_mutex_);
	status_.position_turns = pos;
	status_.velocity_turns_s = vel;
	status_.chain_position_mm = turns_to_chain_mm(pos);
	status_.chain_velocity_mps = turns_s_to_chain_mps(vel);
	pos_turns = pos;
	vel_turns_s = vel;
	return true;
}

bool PreyMotor::try_sample_velocity_turns_s(float& vel_turns_s, int timeout_ms) const {
	// const path for calibrator/hunt_sim — same CAN read, updates cached status.
	if (!is_connected()) {
		return false;
	}
	float pos = 0.0f;
	float vel = 0.0f;
	if (!can_.get_encoder_estimates(pos, vel, timeout_ms)) {
		return false;
	}
	std::lock_guard<std::mutex> lock(status_mutex_);
	status_.position_turns = pos;
	status_.velocity_turns_s = vel;
	status_.chain_position_mm = turns_to_chain_mm(pos);
	status_.chain_velocity_mps = turns_s_to_chain_mps(vel);
	vel_turns_s = vel;
	return true;
}

bool PreyMotor::enter_velocity_mode(int timeout_ms) {
	if (!status_.connected) {
		return false;
	}
	// Stale disarm/watchdog faults block CLOSED_LOOP until cleared.
	can_.clear_errors();
	return can_.enter_velocity_mode(timeout_ms);
}

bool PreyMotor::run_full_calibration(int timeout_ms) {
	if (!status_.connected) {
		return false;
	}
	return can_.run_full_calibration(timeout_ms);
}

bool PreyMotor::set_velocity_turns_s(float turns_s) {
	if (!status_.connected) {
		return false;
	}
	// Skip refresh on the command path so spin loops can feed the watchdog.
	return can_.set_input_velocity(turns_s, 0.0f);
}

bool PreyMotor::command_turns_s(float turns_s) {
	// Same entry point as --vel-turns-s (MotionPlanner blocking execute).
	return set_velocity_turns_s(turns_s);
}

bool PreyMotor::prepare_velocity_move() {
	return assert_velocity_control();
}

bool PreyMotor::try_sample_velocity_turns_s(float& vel_turns_s) {
	return try_sample_velocity_turns_s(vel_turns_s, /*timeout_ms=*/0);
}

float PreyMotor::read_position_turns() const {
	refresh_status();
	std::lock_guard<std::mutex> lock(status_mutex_);
	return status_.position_turns;
}

bool PreyMotor::read_axis_state(uint32_t& axis_error, uint32_t& axis_state) const {
	if (!status_.connected) {
		return false;
	}
	return can_.get_axis_state(axis_error, axis_state);
}

bool PreyMotor::assert_velocity_control() {
	if (!status_.connected) {
		return false;
	}
	// Why: lab saw state=8/err=0 with sample_vel≈0 — re-assert mode+limits
	// without cycling IDLE (GUI can leave vel_limit=0 or wrong control_mode).
	return can_.refresh_velocity_limits();
}

bool PreyMotor::try_get_iq(float& iq_setpoint, float& iq_measured, int timeout_ms) const {
	if (!is_connected()) {
		return false;
	}
	return can_.get_iq(iq_setpoint, iq_measured, timeout_ms);
}

bool PreyMotor::try_get_active_errors(uint32_t& active_errors, uint32_t& disarm_reason,
	int timeout_ms) const
{
	if (!is_connected()) {
		return false;
	}
	return can_.get_active_errors(active_errors, disarm_reason, timeout_ms);
}
