#include "motor/chase_policy.h"

#include <algorithm>
#include <cmath>

#include "motor/lab_motion_limits.h"
#include "motor/motion_planner.h"

namespace {

float clamp01(float v) {
	return std::max(0.0f, std::min(1.0f, v));
}

float angle_diff_deg(float a, float b) {
	float d = std::fabs(a - b);
	while (d > 180.0f) {
		d -= 360.0f;
	}
	return std::fabs(d);
}

float clampf(float v, float lo, float hi) {
	return std::max(lo, std::min(hi, v));
}

// Build a flee plan the motor can execute: checklist for feasibility, then
// runtime profile (floor speed + spin-up lead-in) for start_plan/tick.
ChainMovePlan plan_feasible_flee(float flee_mm, float speed_mps, float accel_mps2,
	float mm_per_turn, int flee_sign, const ChasePolicyConfig& cfg)
{
	const float speed_abs = std::max(0.99f, std::fabs(speed_mps));
	const float speed_mmps = speed_abs * 1000.0f;
	float dist = flee_mm;
	int duration_ms = static_cast<int>(std::lround(
		(LabMotionLimits::kSpinupLeadInS + std::fabs(dist) / speed_mmps)
		* 1000.0f));
	// Never arm a flee shorter than spin-up + margin (~1.5 s).
	if (duration_ms < 1500) {
		duration_ms = 1500;
	}

	for (int attempt = 0; attempt < 8; ++attempt) {
		ChainMovePlan checklist = MotionPlanner::plan_distance_mm_in_time(
			mm_per_turn, flee_sign, dist, duration_ms, accel_mps2);
		if (checklist.feasible) {
			// Why: checklist peaks can be << 1.5 turns/s; runtime lifts floor
			// and sizes hold for distance + spin-up.
			return MotionPlanner::runtime_plan_distance_mm_in_time(mm_per_turn,
				flee_sign, dist, duration_ms, accel_mps2);
		}
		// Relax: longer time first, then shorter distance.
		if (!checklist.accel_ok || !checklist.vel_limit_ok) {
			duration_ms = static_cast<int>(duration_ms * 1.5f);
			if (duration_ms > 10000) {
				duration_ms = 10000;
			}
		}
		if (std::fabs(dist) > cfg.min_flee_mm) {
			dist *= 0.75f;
			if (std::fabs(dist) < cfg.min_flee_mm) {
				dist = std::copysign(cfg.min_flee_mm, flee_mm);
			}
		}
	}
	return MotionPlanner::runtime_plan_distance_mm_in_time(mm_per_turn,
		flee_sign, dist, duration_ms, accel_mps2);
}

} // namespace

ChaseDecision compute_chase_decision(const TrackingFrame& scene,
	const ChasePolicyConfig& cfg, int flee_direction_sign, float mm_per_turn)
{
	ChaseDecision out;
	out.decision_time_ns = scene.host_time_ns;
	out.flee_direction_sign = flee_direction_sign;

	if (scene.trial_phase != TrialPhase::Running) {
		out.reason = "trial_not_running";
		return out;
	}
	if (!scene.both_valid()) {
		out.reason = "tracks_invalid";
		return out;
	}
	if (scene.quality.ferret_confidence < 0.3f || scene.quality.prey_confidence < 0.3f) {
		out.reason = "low_track_confidence";
		return out;
	}

	out.enable_motion = true;
	const float dist = scene.distance_mm;
	float dist_threat = 1.0f;
	if (dist > cfg.threat_distance_mm) {
		const float span = std::max(1.0f, cfg.creep_distance_mm - cfg.threat_distance_mm);
		dist_threat = 1.0f - clamp01((dist - cfg.threat_distance_mm) / span);
	}

	float cone_threat = 1.0f;
	const float heading_delta = angle_diff_deg(scene.ferret.state.direction_deg,
		scene.bearing_deg);
	if (heading_delta > cfg.cone_half_angle_deg) {
		cone_threat = 1.0f - clamp01((heading_delta - cfg.cone_half_angle_deg) / 90.0f);
	}

	float approach_threat = scene.closing_speed_mm_s > 0.0f ? 1.0f : 0.2f;
	out.threat = clamp01(dist_threat * cone_threat * approach_threat);

	const float speed_span = cfg.max_chain_speed_mps - cfg.min_chain_speed_mps;
	out.target_chain_speed_mps = cfg.min_chain_speed_mps + out.threat * speed_span;
	out.target_chain_speed_mps *= static_cast<float>(flee_direction_sign);

	// High threat → discrete MotionPlanner flee from gap + closing speed.
	if (out.threat > cfg.flee_threat_threshold && mm_per_turn > 1e-3f) {
		const float closing = std::max(0.0f, scene.closing_speed_mm_s);
		float flee_mag = cfg.flee_gap_gain * std::max(0.0f, dist)
			+ cfg.flee_speed_gain * closing;
		flee_mag = clampf(flee_mag, cfg.min_flee_mm, cfg.max_flee_mm);
		const float flee_mm = flee_mag * static_cast<float>(flee_direction_sign);
		const float speed_mps = std::fabs(out.target_chain_speed_mps);
		ChainMovePlan plan = plan_feasible_flee(flee_mm, speed_mps,
			cfg.flee_accel_mps2, mm_per_turn, flee_direction_sign, cfg);
		if (plan.feasible) {
			out.use_planned_flee = true;
			out.planned_flee = plan;
			out.reason = "flee_plan";
			return out;
		}
		out.reason = "flee_infeasible_creep";
		return out;
	}

	out.reason = out.threat > 0.05f ? "chase" : "creep";
	return out;
}

MotorCommand chase_decision_to_prey_command(const ChaseDecision& decision) {
	MotorCommand cmd;
	cmd.command_time_ns = decision.decision_time_ns;
	if (!decision.enable_motion) {
		cmd.mode = MotorMode::Idle;
		return cmd;
	}
	cmd.mode = MotorMode::Velocity;
	cmd.velocity_mps = decision.target_chain_speed_mps;
	return cmd;
}
