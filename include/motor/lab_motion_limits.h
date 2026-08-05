#pragma once

// Lab chain-drive characterization (ODrive node 62, chain_mm_per_motor_turn
// ≈ 660.4). Measured 2026-07-31 with closed-loop distance stop.
//
// Single source of truth for MotionPlanner / chase defaults / README.
// Re-sweep with test_odrive_move --via-planner if the sprocket or load changes.
namespace LabMotionLimits {

// sign(omega) with a small dead zone around zero, used by the physical model
// torque = J*alpha + B*omega + tau_c*sign(omega). Shared by PreyMotor's
// torque_ff feed-forward (compute_torque_ff_nm) and
// test_motor_inertia_calibration's dynamic-fit regression (fit_dynamic_model)
// so both apply the exact same rule — the fitted tau_c only means what
// production's feed-forward expects if the two agree bit-for-bit.
constexpr double kSignOmegaEpsilon = 1e-3;
inline double sign_omega(double omega_rad) {
	if (omega_rad > kSignOmegaEpsilon) {
		return 1.0;
	}
	if (omega_rad < -kSignOmegaEpsilon) {
		return -1.0;
	}
	return 0.0;
}

// Below this Set_Input_Vel, the axis often sits in closed-loop with ~0 motion.
constexpr float kMinViableTurnsPerS = 1.5f;

// Breakaway kick: PreyMotor commands a fixed, deliberately-oversized speed
// when starting from rest into a low target speed (to break static
// friction), then swaps to the literal target once measured velocity shows
// real progress — not on a fixed timer, since how long breakaway actually
// takes depends on real hardware/load, not a guess. Below kMinViableTurnsPerS
// this is close to necessary; up to kKickMaxTargetTurnsS it still measurably
// helps. Tune against real hardware, not derived from any measurement.
constexpr float kKickMaxTargetTurnsS = 3.0f;
// Only kicks when starting from near-rest (a genuine breakaway), not for
// every intermediate step of an already-moving low-speed trajectory —
// otherwise a deliberate slow multi-step ramp would jerk at each step.
constexpr float kKickFromRestTurnsS = 0.3f;
// Flat commanded speed during the kick, regardless of target — deliberately
// well above any target in the kick-eligible range, since the cutoff below
// (not this magnitude) determines how far it actually goes.
constexpr float kKickFixedTurnsS = 5.0f;
// Swap from the kick to the literal target once |measured| reaches this
// fraction of |target| (0.8 = stop kicking once within 20% of target).
constexpr float kKickCutoffFraction = 0.8f;
// Safety fallback only — ends the kick even if velocity feedback never
// shows the cutoff fraction (stale/missing telemetry), so a bad read can't
// pin the motor at kKickFixedTurnsS indefinitely.
constexpr float kKickMaxDurationS = 2.0f;
// Below this, two successive commands count as "the same target" (floating-
// point jitter from repeated unit conversions, not a genuine change) — keeps
// a repeated Set_Input_Vel(same target) from re-evaluating and cancelling an
// in-progress kick before it reaches the cutoff.
constexpr float kKickRetriggerDeltaTurnsS = 0.05f;

// Time for sample_vel to approach a step command at ~1.5–1.8 turns/s.
constexpr float kSpinupLeadInS = 1.2f;

// Coast model after Set_Input_Vel(0): lead_mm ≈ |vel| * mm_per_turn * this.
constexpr float kCoastLeadS = 0.04f;
constexpr float kCoastLeadMinMm = 10.0f;
constexpr float kCoastLeadMaxMm = 40.0f;

// Extra execute timeout beyond the plan window (slow spool-up safety).
constexpr float kClosedLoopTimeoutSlackS = 2.0f;

// Documented step intent (no linear ramp through the dead zone).
constexpr float kFloorRampAccelMps2 = 50.0f;

// --- Distance burst guidance (isolated moves, ~0.5 s settle between) ---
// Forward/reverse closed-loop sweeps hit ~±5% from 80 mm up; 40–60 mm also
// tracked but are near coast-lead size. Prefer ≥100 mm for chase flees.
constexpr float kShortestReliableBurstMm = 80.0f;
constexpr float kComfortableMinBurstMm = 100.0f;

// Back-to-back flees with <~100 ms gap overshoot (~20–35 mm on 120/140 mm).
// ≥200–300 ms idle (or hunt_event_min_interval_ms) clears residual coast.
constexpr int kBackToBackMinGapMs = 250;

// Conservative live-chase floor (JSON / ChasePolicyConfig default).
constexpr float kChaseMinFleeMm = 200.0f;
constexpr int kChaseHuntIntervalMs = 1500;

} // namespace LabMotionLimits
