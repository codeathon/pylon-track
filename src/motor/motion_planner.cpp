#include "motor/motion_planner.h"

#include <algorithm>
#include <cmath>
#include <thread>

#include "motor/prey_motor.h"
#include "log/logger.h"

namespace {
// Below this the motor doesn't reliably produce enough torque to actually
// turn (observed on the rig) — a profile whose peak never clears this speed
// commands motion that never happens. Native units (turns/s), not chain
// mm/s, since this is a property of the motor itself, independent of
// whatever sprocket/chain ratio it happens to be driving.
constexpr float kMinViableTurnsPerS = 1.5f;
// Ramp rate used only to reach kMinViableTurnsPerS in the floor case below —
// separate from (and always faster than) the caller's max_accel_mps2, which
// that case overrides. Must be fast: crawling up to the floor at a gentle
// rate spends most of the ramp sitting in the exact low-speed range that
// doesn't produce torque, so it can stall out before ever reaching a speed
// that actually turns the chain. 50 m/s^2 matches the --max-accel value
// that was confirmed on the rig to move the chain.
constexpr float kFloorRampAccelMps2 = 50.0f;
} // namespace

// Build an executable profile with min-viable floor + real (fast) accel ramp.
// Why: plan_chain_move is a feasibility checklist; runtime also lifts peaks
// that would sit in the dead zone and ramps quickly so torque actually appears.
ChainMovePlan MotionPlanner::runtime_plan_distance_mm_in_time(
	const PreyMotor& motor, float distance_mm, int duration_ms,
	float max_accel_mps2)
{
	ChainMovePlan p;
	p.distance_mm = distance_mm;
	p.duration_ms = duration_ms;
	p.mm_per_turn = motor.mm_per_turn();
	p.chain_direction_sign = (motor.config().chain_direction_sign < 0) ? -1 : 1;
	p.scale_ok = motor.has_valid_chain_scale();
	p.timing_ok = duration_ms > 0 && std::fabs(distance_mm) >= 1e-3f;
	if (!p.scale_ok || !p.timing_ok) {
		p.feasible = false;
		p.summary = "[!!] Invalid scale or timing for runtime plan\n";
		return p;
	}

	const float distance_m = distance_mm / 1000.0f;
	const float duration_s = static_cast<float>(duration_ms) / 1000.0f;
	const float accel = (max_accel_mps2 > 0.0f) ? max_accel_mps2 : 0.5f;
	p.accel_mps2 = accel;
	p.duration_s = duration_s;
	p.avg_speed_mps = distance_m / duration_s;

	// Triangle peak covering |x| in T; cap by accel so accel+decel fit the window.
	float peak_abs = 2.0f * std::fabs(distance_m) / duration_s;
	const float peak_accel_cap = accel * duration_s * 0.5f;
	p.min_accel_mps2 = 4.0f * std::fabs(distance_m) / (duration_s * duration_s);
	p.accel_ok = peak_abs <= peak_accel_cap + 1e-6f;
	if (!p.accel_ok) {
		log_info("motor", "Accel limit caps peak to "
			+ std::to_string(peak_accel_cap)
			+ " m/s — raise max_accel_mps2 for short/fast moves");
		peak_abs = peak_accel_cap;
	}

	float ramp_accel = accel;
	const float min_viable_mps = kMinViableTurnsPerS * p.mm_per_turn / 1000.0f;
	if (peak_abs > 1e-6f && peak_abs < min_viable_mps) {
		log_info("motor", "Target peak " + std::to_string(peak_abs)
			+ " m/s is below this motor's minimum viable speed (~"
			+ std::to_string(min_viable_mps) + " m/s / "
			+ std::to_string(kMinViableTurnsPerS) + " turns/s) — moving at "
			"the floor speed instead of commanding a speed the motor can't "
			"sustain");
		peak_abs = min_viable_mps;
		ramp_accel = kFloorRampAccelMps2;
		p.accel_mps2 = ramp_accel;
	}

	// Ramp at real accel (fast), cruise for remaining distance, clamp to time left.
	const float accel_time = (peak_abs > 1e-6f) ? (peak_abs / ramp_accel) : 0.0f;
	const float time_left = std::max(0.0f, duration_s - 2.0f * accel_time);
	const float distance_cruise_time = (peak_abs > 1e-6f)
		? std::max(0.0f, std::fabs(distance_m) / peak_abs - accel_time)
		: 0.0f;
	const float cruise_time = std::min(time_left, distance_cruise_time);

	p.peak_speed_mps = std::copysign(peak_abs, distance_m);
	p.accel_time_s = accel_time;
	p.cruise_time_s = cruise_time;
	// Tick Done when the profile ends (may finish early after floor lift).
	p.duration_s = 2.0f * accel_time + cruise_time;
	p.duration_ms = static_cast<int>(std::lround(p.duration_s * 1000.0f));
	p.expected_distance_mm = std::copysign(
		1000.0f * peak_abs * (accel_time + cruise_time), distance_mm);
	p.peak_turns_s = (p.peak_speed_mps * 1000.0f / p.mm_per_turn)
		* static_cast<float>(p.chain_direction_sign);
	p.vel_limit_ok = std::fabs(p.peak_turns_s) <= p.odrive_vel_limit_turns_s + 1e-3f;
	p.feasible = p.scale_ok && p.timing_ok && p.vel_limit_ok;
	p.summary = "runtime plan peak=" + std::to_string(p.peak_turns_s) + " turns/s\n";
	return p;
}

MotionPlanner::~MotionPlanner() {
	// Why: no further ticks after destroy — clear active so is_busy() is false.
	cancel_.store(true);
	active_.store(false);
}

void MotionPlanner::cancel() {
	// Cooperative: leave active_ set so the next tick() can stop the motor
	// and return MoveTick::Cancelled (unit tests + Ctrl+C during execute_plan).
	cancel_.store(true);
}

ChainMovePlan MotionPlanner::plan_distance_mm_in_time(const PreyMotor& motor,
	float distance_mm, int duration_ms, float max_accel_mps2,
	float odrive_vel_limit_turns_s)
{
	return plan_distance_mm_in_time(motor.mm_per_turn(),
		motor.config().chain_direction_sign, distance_mm, duration_ms,
		max_accel_mps2, odrive_vel_limit_turns_s);
}

ChainMovePlan MotionPlanner::plan_distance_mm_in_time(float mm_per_turn,
	int chain_direction_sign, float distance_mm, int duration_ms,
	float max_accel_mps2, float odrive_vel_limit_turns_s)
{
	return plan_chain_move(distance_mm, duration_ms, max_accel_mps2,
		mm_per_turn, chain_direction_sign, odrive_vel_limit_turns_s);
}

bool MotionPlanner::move_distance_mm_in_time(PreyMotor& motor, float distance_mm,
	int duration_ms, float max_accel_mps2, bool require_feasible,
	ChainMovePlan* out_plan)
{
	if (!motor.status().connected) {
		log_error("motor", "move_distance_mm_in_time: motor not connected");
		return false;
	}
	if (!motor.has_valid_chain_scale()) {
		log_error("motor",
			"move_distance_mm_in_time: chain scale is 0 — set "
			"motor.chain_mm_per_motor_turn in arena_experiment.json");
		return false;
	}
	if (duration_ms <= 0) {
		log_error("motor", "move_distance_mm_in_time: invalid duration");
		return false;
	}
	if (std::fabs(distance_mm) < 1e-3f) {
		motor.stop();
		return true;
	}

	// Why: runtime plan applies 1.5 turns/s floor + fast ramp from web-ui-odrive.
	ChainMovePlan plan = runtime_plan_distance_mm_in_time(motor, distance_mm,
		duration_ms, max_accel_mps2);
	if (out_plan) {
		*out_plan = plan;
	}
	if (require_feasible && !plan.feasible) {
		log_error("motor", "Move refused — plan not feasible:\n" + plan.summary);
		return false;
	}

	log_info("motor", "Move " + std::to_string(static_cast<int>(distance_mm))
		+ " mm in " + std::to_string(duration_ms) + " ms (peak "
		+ std::to_string(plan.peak_speed_mps) + " m/s ≈ "
		+ std::to_string(plan.peak_turns_s) + " turns/s)");
	return execute_plan(motor, plan);
}

float MotionPlanner::sample_speed_mps(float elapsed_s) const {
	const float duration_s = plan_.duration_s;
	const float peak_speed = plan_.peak_speed_mps;
	const float accel_time = plan_.accel_time_s;
	const float cruise_time = plan_.cruise_time_s;
	if (elapsed_s >= duration_s) {
		return 0.0f;
	}
	if (accel_time > 1e-6f && elapsed_s < accel_time) {
		return peak_speed * (elapsed_s / accel_time);
	}
	if (elapsed_s < accel_time + cruise_time) {
		return peak_speed;
	}
	if (accel_time > 1e-6f) {
		const float decel_t = elapsed_s - accel_time - cruise_time;
		const float decel_frac = std::min(1.0f, decel_t / accel_time);
		return peak_speed * (1.0f - decel_frac);
	}
	return 0.0f;
}

bool MotionPlanner::start_plan(const ChainMovePlan& plan) {
	// Why: only gate on active_ — execute_plan sets busy_ before calling us.
	if (active_.load()) {
		log_error("motor", "start_plan: planner already active");
		return false;
	}
	if (!plan.scale_ok || !plan.timing_ok) {
		log_error("motor", "start_plan: invalid plan\n" + plan.summary);
		return false;
	}
	plan_ = plan;
	cancel_.store(false);
	start_time_ = std::chrono::steady_clock::now();
	active_.store(true);
	log_info("motor", "start_plan " + std::to_string(static_cast<int>(plan.distance_mm))
		+ " mm in " + std::to_string(plan.duration_ms) + " ms (peak "
		+ std::to_string(plan.peak_turns_s) + " turns/s)");
	return true;
}

MoveTick MotionPlanner::tick(IMotor& motor) {
	if (!active_.load()) {
		return MoveTick::Idle;
	}
	if (cancel_.load()) {
		motor.stop();
		active_.store(false);
		return MoveTick::Cancelled;
	}
	if (!motor.status().connected) {
		motor.stop();
		active_.store(false);
		return MoveTick::Cancelled;
	}

	const float elapsed_s = std::chrono::duration<float>(
		std::chrono::steady_clock::now() - start_time_).count();
	if (elapsed_s >= plan_.duration_s || std::fabs(plan_.distance_mm) < 1e-3f) {
		motor.stop();
		active_.store(false);
		return MoveTick::Done;
	}

	MotorCommand cmd;
	cmd.mode = MotorMode::Velocity;
	cmd.velocity_mps = sample_speed_mps(elapsed_s);
	motor.apply(cmd);
	return MoveTick::Active;
}

bool MotionPlanner::execute_plan(IMotor& motor, const ChainMovePlan& plan)
{
	if (busy_.exchange(true)) {
		log_error("motor", "MotionPlanner already busy");
		return false;
	}
	struct ScopeExit {
		std::atomic<bool>& busy;
		~ScopeExit() { busy.store(false); }
	} guard{busy_};

	if (!motor.status().connected) {
		log_error("motor", "execute_plan: motor not connected");
		return false;
	}
	// start_plan ignores busy_ so this blocking path can arm the move.
	if (!start_plan(plan)) {
		return false;
	}

	while (true) {
		const MoveTick t = tick(motor);
		if (t == MoveTick::Done) {
			log_info("motor", "Move complete");
			return true;
		}
		if (t == MoveTick::Cancelled || t == MoveTick::Idle) {
			log_info("motor", "Move cancelled");
			return false;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
}
