#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>

#include "motor/chain_move_plan.h"
#include "motor/i_motor.h"

class PreyMotor;

enum class MoveTick : uint8_t {
	Idle,
	Active,
	Done,
	Cancelled
};

// High-level chain move API: plan a distance-in-time profile, then execute it.
// Blocking execute_plan loops tick(); chase uses start_plan + tick at 50 Hz.
class MotionPlanner {
public:
	~MotionPlanner();

	// Build a feasibility plan from motor scale/sign (no CAN motion).
	static ChainMovePlan plan_distance_mm_in_time(const PreyMotor& motor,
		float distance_mm, int duration_ms, float max_accel_mps2 = 5.0f,
		float odrive_vel_limit_turns_s = 10.0f);

	// Plan without a PreyMotor instance (chase policy / offline / unit tests).
	static ChainMovePlan plan_distance_mm_in_time(float mm_per_turn,
		int chain_direction_sign, float distance_mm, int duration_ms,
		float max_accel_mps2 = 5.0f, float odrive_vel_limit_turns_s = 10.0f);

	// Plan + blocking execute. If require_feasible, refuses when not feasible.
	bool move_distance_mm_in_time(PreyMotor& motor, float distance_mm,
		int duration_ms, float max_accel_mps2 = 5.0f,
		bool require_feasible = true, ChainMovePlan* out_plan = nullptr);

	// Arm a non-blocking move (chase control loop). Fails if already active.
	bool start_plan(const ChainMovePlan& plan);
	// Advance trapezoid one step; call ~50 Hz. Stops motor on Done/Cancelled.
	// IMotor so unit tests can use a fake without SocketCAN.
	MoveTick tick(IMotor& motor);
	bool is_move_active() const { return active_.load(); }

	// Blocking execute (test_odrive_move) — loops start_plan + tick.
	bool execute_plan(IMotor& motor, const ChainMovePlan& plan);

	// Cooperative cancel: next tick() → Cancelled + motor.stop().
	// Callers that will not tick again should tick once or force-idle via dtor.
	void cancel();
	bool is_busy() const { return busy_.load() || active_.load(); }

private:
	float sample_speed_mps(float elapsed_s) const;

	std::atomic<bool> busy_{false};   // blocking execute_plan in progress
	std::atomic<bool> active_{false}; // non-blocking start_plan in progress
	std::atomic<bool> cancel_{false};

	ChainMovePlan plan_{};
	std::chrono::steady_clock::time_point start_time_{};
};
