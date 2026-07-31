#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include "motor/shuttle_motor.h"
#include "vision/arena_mask.h"

// Chase policy gains for Phase 1; loaded early so config path is wired in Phase 0.
struct ChasePolicyConfig {
	// Lab floor ~1.0 m/s (1.5 turns/s); below this Set_Input_Vel does not move.
	float min_chain_speed_mps = 1.0f;
	float max_chain_speed_mps = 1.4f;
	float cone_half_angle_deg = 45.0f;
	float threat_distance_mm = 800.0f;
	float creep_distance_mm = 2000.0f;
	// Planned flee bursts (MotionPlanner) when threat exceeds threshold.
	float flee_threat_threshold = 0.35f;
	// Short flees die in spin-up (~1.2 s); keep bursts long enough to travel.
	float min_flee_mm = 200.0f;
	float max_flee_mm = 600.0f;
	float flee_gap_gain = 0.25f;   // fraction of current ferret–prey gap
	float flee_speed_gain = 0.15f; // seconds of closing-speed look-ahead
	float flee_accel_mps2 = 50.0f;
	// After a flee completes, re-arm hunt events while threat stays high.
	int hunt_event_min_interval_ms = 1500;
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
	ShuttleMotorConfig shuttle;
	VisionConfig vision;
	double trial_timeout_s = 120.0;
};

bool load_arena_experiment_config(const std::string& path, ArenaExperimentConfig& out);

// Merge-write vision masks without clobbering other JSON sections.
bool save_arena_vision_masks(const std::string& path, const ArenaMaskConfig& mask);

// Merge-write motor chain calibration fields.
bool save_motor_calibration(const std::string& path, const MotorConfig& motor);
