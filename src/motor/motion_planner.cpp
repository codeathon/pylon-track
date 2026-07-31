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

	const float min_viable_mps = kMinViableTurnsPerS * p.mm_per_turn / 1000.0f;
	if (peak_abs > 1e-6f && peak_abs < min_viable_mps) {
		log_info("motor", "Target peak " + std::to_string(peak_abs)
			+ " m/s is below this motor's minimum viable speed (~"
			+ std::to_string(min_viable_mps) + " m/s / "
			+ std::to_string(kMinViableTurnsPerS) + " turns/s) — moving at "
			"the floor speed instead of commanding a speed the motor can't "
			"sustain");
		peak_abs = min_viable_mps;
	}

	// Why: step/coast/stop — no linear ramp. This motor produces no motion while
	// Set_Input_Vel is below ~1.5 turns/s, so a 20–50 ms ramp through that band
	// (even at 50 m/s²) still wastes the first commands at dead-zone speeds.
	// Hold peak for distance/peak (clamped to the window), then command 0.
	const float accel_time = 0.0f;
	const float cruise_time = (peak_abs > 1e-6f)
		? std::min(duration_s, std::fabs(distance_m) / peak_abs)
		: 0.0f;
	p.accel_mps2 = kFloorRampAccelMps2; // documented intent: instant rise to peak

	p.peak_speed_mps = std::copysign(peak_abs, distance_m);
	p.accel_time_s = accel_time;
	p.cruise_time_s = cruise_time;
	// Why: keep the *requested* duration as the execute/tick window (matches the
	// pre-chase loop proven on the rig). After accel+cruise+decel the sampler
	// returns 0 and we keep feeding Set_Input_Vel(0) until duration elapses —
	// shortening the window here caused short moves to end before the motor
	// had spun up.
	p.duration_s = duration_s;
	p.duration_ms = duration_ms;
	p.expected_distance_mm = std::copysign(
		1000.0f * peak_abs * (accel_time + cruise_time), distance_mm);
	p.peak_turns_s = (p.peak_speed_mps * 1000.0f / p.mm_per_turn)
		* static_cast<float>(p.chain_direction_sign);
	p.vel_limit_ok = std::fabs(p.peak_turns_s) <= p.odrive_vel_limit_turns_s + 1e-3f;
	p.feasible = p.scale_ok && p.timing_ok && p.vel_limit_ok
		&& std::fabs(p.peak_turns_s) + 1e-3f >= kMinViableTurnsPerS;
	p.summary = "runtime plan peak=" + std::to_string(p.peak_turns_s)
		+ " turns/s  accel_t=" + std::to_string(accel_time)
		+ "s  cruise_t=" + std::to_string(cruise_time)
		+ "s  window=" + std::to_string(duration_ms) + "ms\n";
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
	// Why: elapsed==0 makes the linear ramp command exactly 0 for the first
	// Set_Input_Vel; advance a millisecond so the first tick produces torque.
	const float t = std::max(elapsed_s, 0.001f);
	if (accel_time > 1e-6f && t < accel_time) {
		return peak_speed * (t / accel_time);
	}
	if (t < accel_time + cruise_time) {
		return peak_speed;
	}
	if (accel_time > 1e-6f) {
		const float decel_t = t - accel_time - cruise_time;
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
	// Why: is_connected() must not touch CAN. PreyMotor::status() blocks up to
	// rx_timeout (~200 ms) waiting on encoder frames; short flees then hit
	// duration_s and Done before any Set_Input_Vel is applied.
	if (!motor.is_connected()) {
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

	// Non-blocking connect check — never call status() here (CAN encoder wait).
	if (!motor.is_connected()) {
		log_error("motor", "execute_plan: motor not connected");
		return false;
	}
	if (!plan.scale_ok || !plan.timing_ok) {
		log_error("motor", "execute_plan: invalid plan\n" + plan.summary);
		return false;
	}

	// Why: blocking tests use a direct Set_Input_Vel loop. Prefer PreyMotor's
	// raw turns/s API (same path as --vel-turns-s) so mm→turns conversion and
	// CAN send failures are visible in the log.
	plan_ = plan;
	cancel_.store(false);
	active_.store(true);
	const auto start = std::chrono::steady_clock::now();
	int applies = 0;
	int send_fail = 0;
	float peak_cmd_turns_s = 0.0f;
	float last_cmd_turns_s = 0.0f;
	auto* prey = dynamic_cast<PreyMotor*>(&motor);

	while (!cancel_.load()) {
		const float elapsed_s = std::chrono::duration<float>(
			std::chrono::steady_clock::now() - start).count();
		if (elapsed_s >= plan_.duration_s) {
			break;
		}
		const float speed_mps = sample_speed_mps(elapsed_s);
		if (prey) {
			last_cmd_turns_s = prey->chain_mps_to_turns_s(speed_mps);
			peak_cmd_turns_s = std::max(peak_cmd_turns_s,
				std::fabs(last_cmd_turns_s));
			if (!prey->set_velocity_turns_s(last_cmd_turns_s)) {
				++send_fail;
			}
			if (applies < 3) {
				log_info("motor", "cmd #" + std::to_string(applies)
					+ " Set_Input_Vel=" + std::to_string(last_cmd_turns_s)
					+ " turns/s");
			}
		} else {
			MotorCommand cmd;
			cmd.mode = MotorMode::Velocity;
			cmd.velocity_mps = speed_mps;
			motor.apply(cmd);
			last_cmd_turns_s = speed_mps; // FakeMotor: m/s as placeholder
		}
		++applies;
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}

	motor.stop();
	active_.store(false);
	if (cancel_.load()) {
		log_info("motor", "Move cancelled after " + std::to_string(applies)
			+ " commands");
		return false;
	}
	log_info("motor", "Move complete (" + std::to_string(applies)
		+ " commands, peak_cmd=" + std::to_string(peak_cmd_turns_s)
		+ " turns/s, last=" + std::to_string(last_cmd_turns_s)
		+ " turns/s, send_fail=" + std::to_string(send_fail) + ")");
	return applies > 0 && send_fail == 0;
}
