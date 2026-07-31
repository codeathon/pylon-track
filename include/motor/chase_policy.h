#pragma once

#include <cstdint>
#include "experiment/arena_config.h"
#include "motor/chain_move_plan.h"
#include "motor/motor_types.h"
#include "vision/tracking_frame.h"

// Pure policy output — no hardware I/O.
struct ChaseDecision {
	int64_t decision_time_ns = 0;
	float target_chain_speed_mps = 0.0f;
	int flee_direction_sign = 1;
	float threat = 0.0f;
	bool enable_motion = false;
	// When true, controller should start planned_flee (if planner idle).
	bool use_planned_flee = false;
	ChainMovePlan planned_flee;
	const char* reason = "idle";
};

// mm_per_turn + flee_sign needed so planned flees consult MotionPlanner feasibility.
ChaseDecision compute_chase_decision(const TrackingFrame& scene,
	const ChasePolicyConfig& cfg, int flee_direction_sign, float mm_per_turn);

MotorCommand chase_decision_to_prey_command(const ChaseDecision& decision);
