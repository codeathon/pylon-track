#include "motor/motion_planner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "motor/prey_motor.h"
#include "log/logger.h"

MotionPlanner::~MotionPlanner() {
	cancel();
}

void MotionPlanner::cancel() {
	cancel_.store(true);
}

ChainMovePlan MotionPlanner::plan_distance_mm_in_time(const PreyMotor& motor,
	float distance_mm, int duration_ms, float max_accel_mps2,
	float odrive_vel_limit_turns_s)
{
	return plan_chain_move(distance_mm, duration_ms, max_accel_mps2,
		motor.mm_per_turn(), motor.config().chain_direction_sign,
		odrive_vel_limit_turns_s);
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

bool MotionPlanner::execute_plan(PreyMotor& motor, const ChainMovePlan& plan)
{
	if (busy_.exchange(true)) {
		log_error("motor", "MotionPlanner already busy");
		return false;
	}
	cancel_.store(false);

	struct ScopeExit {
		std::atomic<bool>& busy;
		~ScopeExit() { busy.store(false); }
	} guard{busy_};

	if (!motor.status().connected) {
		log_error("motor", "execute_plan: motor not connected");
		return false;
	}
	if (!plan.scale_ok || !plan.timing_ok) {
		log_error("motor", "execute_plan: invalid plan (scale/timing)\n"
			+ plan.summary);
		return false;
	}
	if (std::fabs(plan.distance_mm) < 1e-3f) {
		motor.stop();
		return true;
	}

	const float duration_s = plan.duration_s;
	const float peak_speed = plan.peak_speed_mps;
	const float accel_time = plan.accel_time_s;
	const float cruise_time = plan.cruise_time_s;

	log_info("motor", "Move " + std::to_string(static_cast<int>(plan.distance_mm))
		+ " mm in " + std::to_string(plan.duration_ms) + " ms (peak "
		+ std::to_string(peak_speed) + " m/s ≈ "
		+ std::to_string(plan.peak_turns_s) + " turns/s)");

	const auto start = std::chrono::steady_clock::now();
	while (!cancel_.load()) {
		const float elapsed_s = std::chrono::duration<float>(
			std::chrono::steady_clock::now() - start).count();
		if (elapsed_s >= duration_s) {
			break;
		}

		float speed_mps = 0.0f;
		if (accel_time > 1e-6f && elapsed_s < accel_time) {
			speed_mps = peak_speed * (elapsed_s / accel_time);
		} else if (elapsed_s < accel_time + cruise_time) {
			speed_mps = peak_speed;
		} else if (accel_time > 1e-6f) {
			const float decel_t = elapsed_s - accel_time - cruise_time;
			const float decel_frac = std::min(1.0f, decel_t / accel_time);
			speed_mps = peak_speed * (1.0f - decel_frac);
		}

		MotorCommand cmd;
		cmd.mode = MotorMode::Velocity;
		cmd.velocity_mps = speed_mps;
		motor.apply(cmd);
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}

	motor.stop();
	if (cancel_.load()) {
		log_info("motor", "Move cancelled");
		return false;
	}
	log_info("motor", "Move complete");
	return true;
}
