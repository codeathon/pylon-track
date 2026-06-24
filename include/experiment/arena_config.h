#pragma once

#include <string>

// Chase policy gains for Phase 1; loaded early so config path is wired in Phase 0.
struct ChasePolicyConfig {
	float min_chain_speed_mps = 0.05f;
	float max_chain_speed_mps = 0.8f;
	float cone_half_angle_deg = 45.0f;
	float threat_distance_mm = 800.0f;
	float creep_distance_mm = 2000.0f;
};

struct ArenaExperimentConfig {
	ChasePolicyConfig chase;
	double trial_timeout_s = 120.0;
};

bool load_arena_experiment_config(const std::string& path, ArenaExperimentConfig& out);
