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
	// Cyclic heartbeat is ~100 ms; retry briefly in case the RX buffer was empty
	// on the first poll after open.
	status_.heartbeat_ok = false;
	for (int i = 0; i < 10 && !status_.heartbeat_ok; ++i) {
		status_.heartbeat_ok = can_.check_heartbeat();
	}
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

void PreyMotor::refresh_status() const {
	if (!status_.connected) {
		return;
	}
	float pos = 0.0f;
	float vel = 0.0f;
	if (can_.get_encoder_estimates(pos, vel)) {
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
		const float turns_s = chain_mps_to_turns_s(cmd.velocity_mps);
		can_.set_input_velocity(turns_s, 0.0f);
		refresh_status();
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
	return status_;
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
	const bool ok = can_.set_input_velocity(turns_s, 0.0f);
	refresh_status();
	return ok;
}

float PreyMotor::read_position_turns() const {
	refresh_status();
	return status_.position_turns;
}

bool PreyMotor::read_axis_state(uint32_t& axis_error, uint32_t& axis_state) const {
	if (!status_.connected) {
		return false;
	}
	return can_.get_axis_state(axis_error, axis_state);
}
