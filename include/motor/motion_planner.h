#pragma once

#include <atomic>
#include <cstdint>

#include "motor/chain_move_plan.h"

class PreyMotor;

// High-level chain move API: plan a distance-in-time profile, then execute it.
// Planning uses the same math as execution (ChainMovePlan).
class MotionPlanner {
public:
	~MotionPlanner();

	// Build a feasibility plan from motor scale/sign (no CAN motion).
	static ChainMovePlan plan_distance_mm_in_time(const PreyMotor& motor,
		float distance_mm, int duration_ms, float max_accel_mps2 = 5.0f,
		float odrive_vel_limit_turns_s = 10.0f);

	// Plan + execute. If require_feasible, refuses when accel/scale/vel_limit
	// cannot cover the requested distance in time.
	// out_plan (optional) receives the plan that was used or rejected.
	bool move_distance_mm_in_time(PreyMotor& motor, float distance_mm,
		int duration_ms, float max_accel_mps2 = 5.0f,
		bool require_feasible = true, ChainMovePlan* out_plan = nullptr);

	// Execute a previously built plan (skips re-planning).
	bool execute_plan(PreyMotor& motor, const ChainMovePlan& plan);

	void cancel();
	bool is_busy() const { return busy_.load(); }

private:
	std::atomic<bool> busy_{false};
	std::atomic<bool> cancel_{false};
};
