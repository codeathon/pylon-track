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

bool MotionPlanner::move_distance_mm_in_time(PreyMotor& motor, float distance_mm,
	int duration_ms, float max_accel_mps2)
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
		log_error("motor", "move_distance_mm_in_time: motor not connected");
		return false;
	}
	if (!motor.has_valid_chain_scale()) {
		log_error("motor",
			"move_distance_mm_in_time: chain scale is 0 — set "
			"motor.chain_mm_per_motor_turn in arena_experiment.json (e.g. 157)");
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

	const float distance_m = distance_mm / 1000.0f;
	const float duration_s = static_cast<float>(duration_ms) / 1000.0f;
	const float avg_speed = distance_m / duration_s;
	const float accel = (max_accel_mps2 > 0.0f) ? max_accel_mps2 : 0.5f;

	// Triangular profile when distance is too short for full accel cruise.
	float peak_speed = avg_speed * 2.0f;
	const float min_peak = std::sqrt(accel * std::fabs(distance_m));
	if (peak_speed < min_peak) {
		peak_speed = min_peak;
	}
	peak_speed = std::copysign(peak_speed, distance_m);

	const float accel_time = std::fabs(peak_speed) / accel;
	const float cruise_time = std::max(0.0f, duration_s - 2.0f * accel_time);

	const auto start = std::chrono::steady_clock::now();
	float elapsed_s = 0.0f;

	const float peak_turns_s = motor.chain_mps_to_turns_s(peak_speed);
	log_info("motor", "Move " + std::to_string(static_cast<int>(distance_mm))
		+ " mm in " + std::to_string(duration_ms) + " ms (peak "
		+ std::to_string(peak_speed) + " m/s ≈ "
		+ std::to_string(peak_turns_s) + " turns/s)");

	while (!cancel_.load()) {
		elapsed_s = std::chrono::duration<float>(
			std::chrono::steady_clock::now() - start).count();
		if (elapsed_s >= duration_s) {
			break;
		}

		float speed_mps = 0.0f;
		if (elapsed_s < accel_time) {
			speed_mps = peak_speed * (elapsed_s / accel_time);
		} else if (elapsed_s < accel_time + cruise_time) {
			speed_mps = peak_speed;
		} else {
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
