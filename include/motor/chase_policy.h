#pragma once

#include <cstdint>
#include "experiment/arena_config.h"
#include "motor/motor_types.h"
#include "vision/tracking_frame.h"

// Pure policy output — no hardware I/O.
struct ChaseDecision {
	int64_t decision_time_ns = 0;
	float target_chain_speed_mps = 0.0f;
	int flee_direction_sign = 1;
	float threat = 0.0f;
	bool enable_motion = false;
	const char* reason = "idle";
};

ChaseDecision compute_chase_decision(const TrackingFrame& scene,
	const ChasePolicyConfig& cfg, int flee_direction_sign = 1);

MotorCommand chase_decision_to_prey_command(const ChaseDecision& decision);
