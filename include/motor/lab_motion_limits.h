#pragma once

// Lab chain-drive characterization (ODrive node 62, chain_mm_per_motor_turn
// ≈ 660.4). Measured 2026-07-31 with closed-loop distance stop.
//
// Single source of truth for MotionPlanner / chase defaults / README.
// Re-sweep with test_odrive_move --via-planner if the sprocket or load changes.
namespace LabMotionLimits {

// Below this Set_Input_Vel, the axis often sits in closed-loop with ~0 motion.
constexpr float kMinViableTurnsPerS = 1.5f;

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
