#pragma once

#include <mutex>
#include <string>
#include "motor/i_motor.h"
#include "motor/odrive_can.h"

// ODrive S1 prey/toy chain motor — converts chain mm/s ↔ motor turns/s.
struct PreyMotorConfig {
	std::string can_interface = "can0";
	uint8_t node_id = 0;
	float pulley_radius_m = 0.025f;
	int chain_direction_sign = 1;
	// Optional measured constant; when > 0 overrides pulley_radius for conversions.
	float chain_mm_per_motor_turn = 0.0f;
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

	void refresh_status() const;
};
