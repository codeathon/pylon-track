#include "motor/prey_motor.h"

#include <cmath>
#include "log/logger.h"
#include "motor/lab_motion_limits.h"

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
	have_last_cmd_ = false;
	kicking_ = false;
	pursuing_turns_s_ = 0.0f;
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

float PreyMotor::compute_torque_ff_nm(float target_turns_s) {
	const auto now = std::chrono::steady_clock::now();
	const bool calibrated = cfg_.chain_inertia_kg_m2 > 0.0f;
	float torque_ff = 0.0f;
	if (calibrated && have_last_cmd_) {
		const float dt = std::chrono::duration<float>(now - last_cmd_time_).count();
		// Why: a gap this long means the motor was idle/stopped between
		// commands — a dt-based accel estimate across it would be fictitious,
		// so skip feed-forward for this one command instead of spiking torque_ff.
		constexpr float kMaxValidDtS = 0.15f;
		if (dt > 1e-4f && dt <= kMaxValidDtS) {
			const float accel_turns_s2 = (target_turns_s - last_cmd_turns_s_) / dt;
			const float alpha_rad = accel_turns_s2 * kTwoPi;
			const float omega_rad = target_turns_s * kTwoPi;
			float sign_omega = 0.0f;
			if (omega_rad > 1e-3f) {
				sign_omega = 1.0f;
			} else if (omega_rad < -1e-3f) {
				sign_omega = -1.0f;
			}
			torque_ff = cfg_.chain_inertia_kg_m2 * alpha_rad
				+ cfg_.chain_viscous_friction_nm_s_per_rad * omega_rad
				+ cfg_.chain_static_friction_nm * sign_omega;
		}
	}
	last_cmd_turns_s_ = target_turns_s;
	last_cmd_time_ = now;
	have_last_cmd_ = true;
	return torque_ff;
}

float PreyMotor::apply_kick(float target_turns_s) {
	const auto now = std::chrono::steady_clock::now();
	const bool is_new_pursuit = std::fabs(target_turns_s - pursuing_turns_s_)
		> LabMotionLimits::kKickRetriggerDeltaTurnsS;
	if (is_new_pursuit) {
		// Why: only a genuine breakaway from near-rest should kick — not
		// every intermediate step of an already-moving low-speed trajectory
		// (that would jerk at each step instead of ramping smoothly).
		const bool from_rest = std::fabs(pursuing_turns_s_)
			< LabMotionLimits::kKickFromRestTurnsS;
		const bool ramping_up = std::fabs(target_turns_s) > std::fabs(pursuing_turns_s_);
		kicking_ = from_rest && ramping_up
			&& std::fabs(target_turns_s) > 1e-3f
			&& std::fabs(target_turns_s) < LabMotionLimits::kKickMaxTargetTurnsS;
		if (kicking_) {
			kick_start_time_ = now;
			log_info("motor", "Breakaway kick toward " + std::to_string(target_turns_s)
				+ " turns/s");
		}
		pursuing_turns_s_ = target_turns_s;
	}

	if (!kicking_) {
		return target_turns_s;
	}

	// Cutoff is feedback-driven, not a fixed timer: how long breakaway
	// actually takes depends on real load, not a guess. Reads whatever's
	// currently cached — one iteration stale at worst in the normal
	// set_velocity_turns_s-then-try_sample_velocity_turns_s loop pattern,
	// which is close enough for this decision.
	float measured = 0.0f;
	{
		std::lock_guard<std::mutex> lock(status_mutex_);
		measured = status_.velocity_turns_s;
	}
	const bool reached_cutoff = std::fabs(measured)
		>= std::fabs(target_turns_s) * kick_cutoff_fraction_;
	const float elapsed = std::chrono::duration<float>(now - kick_start_time_).count();
	if (reached_cutoff || elapsed >= LabMotionLimits::kKickMaxDurationS) {
		kicking_ = false;
		return target_turns_s;
	}
	const float sign = (target_turns_s >= 0.0f) ? 1.0f : -1.0f;
	return sign * kick_speed_turns_s_;
}

bool PreyMotor::send_velocity_command(float target_turns_s) {
	// torque_ff comes from the literal target, not the kick-adjusted value —
	// see apply_kick()'s comment on why differentiating across the kick's
	// own jump would inject a spurious braking torque.
	const float torque_ff = compute_torque_ff_nm(target_turns_s);
	const float effective_turns_s = apply_kick(target_turns_s);
	return can_.set_input_velocity(effective_turns_s, torque_ff);
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
		if (!send_velocity_command(turns_s)) {
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
	// Why: next command starts a fresh feed-forward window — a dt-based accel
	// estimate spanning this stop would be fictitious. Same for kick state —
	// the motor is at rest now, so the next low-speed command is a genuine
	// breakaway again.
	have_last_cmd_ = false;
	kicking_ = false;
	pursuing_turns_s_ = 0.0f;
	refresh_status();
}

void PreyMotor::estop() {
	if (!status_.connected) {
		return;
	}
	can_.send_estop();
	can_.set_input_velocity(0.0f, 0.0f);
	have_last_cmd_ = false;
	kicking_ = false;
	pursuing_turns_s_ = 0.0f;
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
	return send_velocity_command(turns_s);
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
