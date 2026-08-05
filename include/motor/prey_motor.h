#pragma once

#include <chrono>
#include <mutex>
#include <string>
#include "motor/i_motor.h"
#include "motor/lab_motion_limits.h"
#include "motor/odrive_can.h"

// ODrive S1 prey/toy chain motor — converts chain mm/s ↔ motor turns/s.
struct PreyMotorConfig {
	std::string can_interface = "can0";
	uint8_t node_id = 0;
	float pulley_radius_m = 0.025f;
	int chain_direction_sign = 1;
	// Optional measured constant; when > 0 overrides pulley_radius for conversions.
	float chain_mm_per_motor_turn = 0.0f;

	// Chain inertia/friction model (test_motor_inertia_calibration output) used
	// as Set_Input_Vel torque feed-forward. chain_inertia_kg_m2 <= 0 means
	// "uncalibrated" — feed-forward is skipped and torque_ff stays 0, same as
	// before this model existed.
	float chain_inertia_kg_m2 = 0.0f;
	float chain_viscous_friction_nm_s_per_rad = 0.0f;
	float chain_static_friction_nm = 0.0f;
	// ODrive motor torque constant (odrivetool: axis0.motor.config.torque_constant).
	float torque_constant_nm_per_a = 0.0827f;
};

class PreyMotor : public IMotor {
public:
	explicit PreyMotor(const PreyMotorConfig& cfg);

	bool connect() override;
	void apply(const MotorCommand& cmd) override;
	void stop() override;
	void estop() override;
	MotorStatus status() const override;
	bool is_connected() const override;
	bool command_turns_s(float turns_s) override;
	bool prepare_velocity_move() override;
	bool try_sample_velocity_turns_s(float& vel_turns_s) override;
	bool try_sample_encoder(float& pos_turns, float& vel_turns_s,
		int timeout_ms = 0) override;

	// Non-blocking / short-timeout encoder sample for telemetry (not the tick path).
	bool try_sample_velocity_turns_s(float& vel_turns_s, int timeout_ms) const;

	bool enter_velocity_mode(int timeout_ms = 10000);
	// Runs AXIS_STATE_FULL_CALIBRATION_SEQUENCE over CAN (motor will move).
	bool run_full_calibration(int timeout_ms = 60000);

	// Raw motor turns/s — used by ODriveCalibrator before chain constants are known.
	bool set_velocity_turns_s(float turns_s);
	float read_position_turns() const;
	// Heartbeat Axis_Error / Axis_State (for setup diagnostics).
	bool read_axis_state(uint32_t& axis_error, uint32_t& axis_state) const;
	// Re-send velocity mode + limits (before a spin if tracking looks dead).
	bool assert_velocity_control();
	bool try_get_iq(float& iq_setpoint, float& iq_measured, int timeout_ms = 0) const;
	bool try_get_active_errors(uint32_t& active_errors, uint32_t& disarm_reason,
		int timeout_ms = 0) const;

	// Overrides the breakaway-kick boost magnitude (LabMotionLimits::
	// kKickBoostTurnsS by default) — used by test_motor_inertia_calibration's
	// kick-tuning phase to find a good value against real hardware before the
	// real sweep runs, rather than trusting an unvalidated constant.
	void set_kick_boost_turns_s(float boost) { kick_boost_turns_s_ = boost; }
	float kick_boost_turns_s() const { return kick_boost_turns_s_; }

	// Chain ↔ motor unit conversions (public for motion planner + calibration).
	float chain_mps_to_turns_s(float chain_mps) const;
	float turns_s_to_chain_mps(float turns_s) const;
	float turns_to_chain_mm(float turns) const;
	float chain_mm_to_turns(float chain_mm) const;
	// False when chain_mm_per_motor_turn and pulley_radius_m are both unset/zero
	// (MotionPlanner would command 0 turns/s and the chain would not move).
	bool has_valid_chain_scale() const { return mm_per_turn_ > 1e-3f; }
	float mm_per_turn() const { return mm_per_turn_; }

	const PreyMotorConfig& config() const { return cfg_; }

private:
	PreyMotorConfig cfg_;
	ODriveCan can_;
	mutable MotorStatus status_;
	// ChaseController's thread and ExperimentStateManager's chase_feed_loop
	// thread both call status()/read_position_turns() concurrently — guards
	// status_ itself (ODriveCan has its own lock for the CAN I/O).
	mutable std::mutex status_mutex_;
	float mm_per_turn_ = 0.0f;

	// Torque feed-forward state — last commanded (not measured) velocity/time,
	// used to estimate the commanded angular accel between successive
	// Set_Input_Vel calls. Only ever touched from the single thread that drives
	// this motor's velocity commands (ChaseController's control loop or a
	// test's main thread), so no lock needed.
	float last_cmd_turns_s_ = 0.0f;
	std::chrono::steady_clock::time_point last_cmd_time_{};
	bool have_last_cmd_ = false;

	// Breakaway-kick state (see LabMotionLimits::kKick*) — same single-thread
	// assumption as the feed-forward state above. pursuing_turns_s_ is the
	// last *literal* target a caller asked for (distinct from
	// last_cmd_turns_s_, which may hold the boosted kick value actually sent).
	float pursuing_turns_s_ = 0.0f;
	bool kicking_ = false;
	std::chrono::steady_clock::time_point kick_start_time_{};
	float kick_boost_turns_s_ = LabMotionLimits::kKickBoostTurnsS;

	void refresh_status() const;
	// torque_ff (N*m) for a Set_Input_Vel(target_turns_s) call, from the
	// chain_inertia_kg_m2/chain_viscous_friction_nm_s_per_rad/
	// chain_static_friction_nm model. Returns 0 when uncalibrated, on the
	// first command, or after a stale gap (motor was idle/stopped).
	float compute_torque_ff_nm(float target_turns_s);
	// Returns the velocity to actually command for this target — boosted
	// past target for LabMotionLimits::kKickDurationS if this is a fresh
	// breakaway from rest into a low target speed, otherwise target itself.
	float apply_kick(float target_turns_s);
	// Shared by set_velocity_turns_s() and apply()'s Velocity branch — the
	// two paths a caller actually commands chain motion through — so kick
	// and feed-forward behavior stay identical everywhere the motor is
	// driven, including test_motor_inertia_calibration.
	bool send_velocity_command(float target_turns_s);
};
