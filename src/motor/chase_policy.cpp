#include "motor/chase_policy.h"

#include <algorithm>
#include <cmath>

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

} // namespace

ChaseDecision compute_chase_decision(const TrackingFrame& scene,
	const ChasePolicyConfig& cfg, int flee_direction_sign)
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
