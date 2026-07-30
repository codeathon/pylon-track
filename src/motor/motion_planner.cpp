#include "motor/motion_planner.h"

#include <algorithm>
#include <cmath>
#include <thread>

#include "motor/prey_motor.h"
#include "log/logger.h"

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
	const ChainMovePlan plan = plan_distance_mm_in_time(motor, distance_mm,
		duration_ms, max_accel_mps2);
	if (out_plan) {
		*out_plan = plan;
	}
	if (require_feasible && !plan.feasible) {
		log_error("motor", "Move refused — plan not feasible:\n" + plan.summary);
		return false;
	}
	if (!plan.accel_ok) {
		log_info("motor", "Executing partial profile (expected "
			+ std::to_string(plan.expected_distance_mm) + " mm)");
	}
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
