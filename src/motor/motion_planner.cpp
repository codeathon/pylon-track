#include "motor/motion_planner.h"

#include <algorithm>
#include <chrono>
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
// separate from the caller's max_accel_mps2, which that case overrides.
constexpr float kFloorRampAccelMps2 = 2.0f;
} // namespace

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
	const float accel = (max_accel_mps2 > 0.0f) ? max_accel_mps2 : 0.5f;

	// Target peak for a bare triangle (no cruise) spanning the FULL duration_s
	// that covers |distance| exactly: v = 2x/t. Cap by accel so accel+decel
	// alone (no cruise) still fit the window when the motor can't ramp that
	// fast (v_max = a*t/2) — the move then finishes short of distance, which
	// is unavoidable given the accel limit.
	float peak_abs = 2.0f * std::fabs(distance_m) / duration_s;
	const float peak_accel_cap = accel * duration_s * 0.5f;
	if (peak_abs > peak_accel_cap) {
		log_info("motor", "Accel limit caps peak to "
			+ std::to_string(peak_accel_cap)
			+ " m/s — raise max_accel_mps2 for short/fast moves");
		peak_abs = peak_accel_cap;
	}

	// Below this the motor doesn't reliably move at all (kMinViableTurnsPerS)
	// — a peak that never clears the floor commands motion that never
	// happens. Ramp to the floor speed instead.
	float ramp_accel = accel;
	const float min_viable_mps = kMinViableTurnsPerS * motor.mm_per_turn() / 1000.0f;
	if (peak_abs > 1e-6f && peak_abs < min_viable_mps) {
		log_info("motor", "Target peak " + std::to_string(peak_abs)
			+ " m/s is below this motor's minimum viable speed (~"
			+ std::to_string(min_viable_mps) + " m/s / "
			+ std::to_string(kMinViableTurnsPerS) + " turns/s) — moving at "
			"the floor speed instead of commanding a speed the motor can't "
			"sustain");
		peak_abs = min_viable_mps;
		ramp_accel = kFloorRampAccelMps2;
	}

	// Ramp at the REAL accel rate (fast) rather than stretching accel_time to
	// fill duration_s — a slow linear ramp spends most of its time crawling
	// through the sub-floor dead zone where the motor produces ~no torque.
	// Reaching peak quickly and holding it there is what actually moves the
	// chain. cruise_time is sized off the remaining DISTANCE (finish early
	// once |distance| is covered) rather than off remaining time, and then
	// clamped to whatever time duration_s actually leaves — if the accel cap
	// forced a lower peak, there may be no time left over at all, and the
	// move simply falls short of the target (already logged above).
	const float accel_time = (peak_abs > 1e-6f) ? (peak_abs / ramp_accel) : 0.0f;
	const float time_left = std::max(0.0f, duration_s - 2.0f * accel_time);
	const float distance_cruise_time = (peak_abs > 1e-6f)
		? std::max(0.0f, std::fabs(distance_m) / peak_abs - accel_time)
		: 0.0f;
	const float cruise_time = std::min(time_left, distance_cruise_time);

	const float peak_speed = std::copysign(peak_abs, distance_m);

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
