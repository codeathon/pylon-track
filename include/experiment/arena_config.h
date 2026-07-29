#pragma once

#include <string>
#include <vector>
#include "motor/trap_door_motor.h"
#include "vision/arena_mask.h"

// Chase policy gains for Phase 1; loaded early so config path is wired in Phase 0.
struct ChasePolicyConfig {
	float min_chain_speed_mps = 0.05f;
	float max_chain_speed_mps = 0.8f;
	float cone_half_angle_deg = 45.0f;
	float threat_distance_mm = 800.0f;
	float creep_distance_mm = 2000.0f;
};

struct MotorConfig {
	std::string can_interface = "can0";
	uint8_t node_id = 0;
	float pulley_radius_m = 0.025f;
	int chain_direction_sign = 1;
	float chain_mm_per_motor_turn = 0.0f;
};

// Animal contour priors — used by ObjectAssociator in a later step.
struct VisionConfig {
	ArenaMaskConfig mask;
	float ferret_area_px_min = 5000.0f;
	float ferret_area_px_max = 60000.0f;
	float prey_area_px_min = 500.0f;
	float prey_area_px_max = 15000.0f;
	float max_compactness = 0.35f;
};

struct ArenaExperimentConfig {
	ChasePolicyConfig chase;
	MotorConfig motor;
	TrapDoorConfig trap_door;
	VisionConfig vision;
	double trial_timeout_s = 120.0;
};

bool load_arena_experiment_config(const std::string& path, ArenaExperimentConfig& out);

// Merge-write vision masks without clobbering other JSON sections.
bool save_arena_vision_masks(const std::string& path, const ArenaMaskConfig& mask);

// Merge-write motor chain calibration fields.
bool save_motor_calibration(const std::string& path, const MotorConfig& motor);
