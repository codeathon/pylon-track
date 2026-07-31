// Offline unit tests for MotionPlanner (no SocketCAN / ODrive).
// Build: make test_motion_planner
// Run:   ./bin/test_motion_planner

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "motor/i_motor.h"
#include "motor/motion_planner.h"

namespace {

int g_failures = 0;

void expect(bool ok, const char* name) {
	if (ok) {
		std::cout << "  PASS  " << name << '\n';
		return;
	}
	std::cout << "  FAIL  " << name << '\n';
	++g_failures;
}

void expect_near(float a, float b, float tol, const char* name) {
	expect(std::fabs(a - b) <= tol, name);
}

// Records apply/stop without touching CAN — lets tick/execute_plan run offline.
class FakeMotor : public IMotor {
public:
	bool connect() override {
		status_.connected = true;
		return true;
	}
	void apply(const MotorCommand& cmd) override {
		++apply_count;
		last_cmd = cmd;
		if (cmd.mode == MotorMode::Velocity) {
			last_velocity_mps = cmd.velocity_mps;
			velocity_samples.push_back(cmd.velocity_mps);
		}
	}
	void stop() override {
		++stop_count;
		last_velocity_mps = 0.0f;
	}
	void estop() override {
		++estop_count;
		last_velocity_mps = 0.0f;
	}
	MotorStatus status() const override { return status_; }

	void set_connected(bool c) { status_.connected = c; }

	MotorStatus status_{};
	MotorCommand last_cmd{};
	float last_velocity_mps = 0.0f;
	int apply_count = 0;
	int stop_count = 0;
	int estop_count = 0;
	std::vector<float> velocity_samples;
};

ChainMovePlan feasible_plan(float distance_mm = 100.0f, int duration_ms = 500) {
	// 157 mm/turn matches lab chain scale; accel high enough for short moves.
	return MotionPlanner::plan_distance_mm_in_time(157.0f, 1, distance_mm,
		duration_ms, 5.0f, 10.0f);
}

void test_plan_feasible_nominal() {
	std::cout << "plan_distance_mm_in_time feasible\n";
	const ChainMovePlan p = feasible_plan(200.0f, 1000);
	expect(p.scale_ok, "scale_ok");
	expect(p.timing_ok, "timing_ok");
	expect(p.accel_ok, "accel_ok");
	expect(p.vel_limit_ok, "vel_limit_ok");
	expect(p.feasible, "feasible");
	expect(p.duration_ms == 1000, "duration_ms");
	expect_near(p.distance_mm, 200.0f, 1e-3f, "distance_mm");
	expect(std::fabs(p.peak_speed_mps) > 0.0f, "peak_speed_nonzero");
}

void test_plan_rejects_zero_scale() {
	std::cout << "plan rejects zero mm_per_turn\n";
	const ChainMovePlan p = MotionPlanner::plan_distance_mm_in_time(0.0f, 1,
		100.0f, 500, 5.0f);
	expect(!p.scale_ok, "scale_not_ok");
	expect(!p.feasible, "not_feasible");
}

void test_plan_rejects_impossible_accel() {
	std::cout << "plan marks infeasible when accel too low for distance/time\n";
	// 400 mm in 50 ms needs huge accel — 0.1 m/s^2 cannot cover it.
	const ChainMovePlan p = MotionPlanner::plan_distance_mm_in_time(157.0f, 1,
		400.0f, 50, 0.1f, 10.0f);
	expect(p.timing_ok, "timing_ok");
	expect(!p.accel_ok, "accel_not_ok");
	expect(!p.feasible, "not_feasible");
}

void test_start_plan_and_tick_idle() {
	std::cout << "tick Idle when no plan armed\n";
	MotionPlanner planner;
	FakeMotor motor;
	motor.connect();
	expect(planner.tick(motor) == MoveTick::Idle, "tick_idle");
	expect(!planner.is_move_active(), "not_active");
}

void test_start_plan_rejects_invalid() {
	std::cout << "start_plan rejects invalid plan\n";
	MotionPlanner planner;
	ChainMovePlan bad;
	bad.distance_mm = 0.0f;
	bad.duration_ms = 0;
	bad.scale_ok = false;
	bad.timing_ok = false;
	expect(!planner.start_plan(bad), "start_rejected");
}

void test_start_plan_rejects_second_while_active() {
	std::cout << "start_plan rejects second plan while active\n";
	MotionPlanner planner;
	FakeMotor motor;
	motor.connect();
	const ChainMovePlan p = feasible_plan(100.0f, 400);
	expect(p.feasible, "plan_feasible");
	if (!p.feasible) {
		return;
	}
	expect(planner.start_plan(p), "first_start");
	expect(planner.is_move_active(), "active");
	expect(!planner.start_plan(p), "second_start_rejected");
	planner.cancel();
	expect(planner.tick(motor) == MoveTick::Cancelled, "cancel_tick");
}

void test_tick_covers_duration() {
	std::cout << "start_plan + tick reaches Done near plan duration\n";
	MotionPlanner planner;
	FakeMotor motor;
	motor.connect();

	// 80 mm in 400 ms needs a_min≈2 m/s^2 — feasible at default 5 m/s^2.
	constexpr int kDurationMs = 400;
	const ChainMovePlan p = feasible_plan(80.0f, kDurationMs);
	expect(p.feasible, "plan_feasible");
	if (!p.feasible) {
		return;
	}
	expect(planner.start_plan(p), "start_plan");

	const auto t0 = std::chrono::steady_clock::now();
	MoveTick last = MoveTick::Idle;
	int active_ticks = 0;
	// Poll faster than 50 Hz so Done is observed soon after duration_s.
	while (true) {
		last = planner.tick(motor);
		if (last == MoveTick::Active) {
			++active_ticks;
		}
		if (last == MoveTick::Done || last == MoveTick::Cancelled
			|| last == MoveTick::Idle) {
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
		const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now() - t0).count();
		if (elapsed_ms > kDurationMs + 500) {
			break;
		}
	}
	const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now() - t0).count();

	expect(last == MoveTick::Done, "ended_done");
	expect(active_ticks > 0, "had_active_ticks");
	expect(motor.apply_count > 0, "applied_velocity");
	expect(motor.stop_count >= 1, "stopped_on_done");
	// Allow scheduling slack: Done should land near the planned duration.
	expect(elapsed_ms + 5 >= kDurationMs, "not_early");
	expect(elapsed_ms <= kDurationMs + 150, "not_too_late");
	expect(!planner.is_move_active(), "idle_after_done");
	expect(!motor.velocity_samples.empty()
		&& std::fabs(motor.velocity_samples.front()) > 0.0f,
		"nonzero_command");
}

void test_cancel_mid_move() {
	std::cout << "cancel mid-move → Cancelled on next tick\n";
	MotionPlanner planner;
	FakeMotor motor;
	motor.connect();

	const ChainMovePlan p = feasible_plan(200.0f, 2000);
	expect(planner.start_plan(p), "start");
	expect(planner.tick(motor) == MoveTick::Active, "active");
	planner.cancel();
	expect(planner.tick(motor) == MoveTick::Cancelled, "cancelled");
	expect(!planner.is_move_active(), "not_active");
	expect(motor.stop_count >= 1, "stopped");
	// Why: after cancel, a new feasible plan must be armable again.
	expect(planner.start_plan(feasible_plan(50.0f, 300)), "rearm");
	planner.cancel();
	expect(planner.tick(motor) == MoveTick::Cancelled, "rearm_cancelled");
	expect(!planner.is_move_active(), "idle_after_rearm_cancel");
}

void test_tick_disconnect_cancels() {
	std::cout << "disconnect during move → Cancelled\n";
	MotionPlanner planner;
	FakeMotor motor;
	motor.connect();
	expect(planner.start_plan(feasible_plan(100.0f, 1000)), "start");
	motor.set_connected(false);
	expect(planner.tick(motor) == MoveTick::Cancelled, "cancelled");
}

void test_execute_plan_blocking() {
	std::cout << "execute_plan loops tick until Done\n";
	MotionPlanner planner;
	FakeMotor motor;
	motor.connect();
	constexpr int kDurationMs = 300;
	const ChainMovePlan p = feasible_plan(60.0f, kDurationMs);
	expect(p.feasible, "feasible");
	if (!p.feasible) {
		return;
	}
	const auto t0 = std::chrono::steady_clock::now();
	expect(planner.execute_plan(motor, p), "execute_ok");
	const auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now() - t0).count();
	expect(elapsed_ms + 5 >= kDurationMs, "duration_floor");
	expect(elapsed_ms <= kDurationMs + 200, "duration_ceil");
	expect(motor.apply_count > 0, "applied");
	expect(!planner.is_busy(), "not_busy");
}

void test_signed_distance_peak_sign() {
	std::cout << "negative distance → negative peak_speed_mps\n";
	const ChainMovePlan p = MotionPlanner::plan_distance_mm_in_time(157.0f, 1,
		-150.0f, 800, 5.0f);
	expect(p.feasible, "feasible");
	expect(p.peak_speed_mps < 0.0f, "negative_peak");
}

} // namespace

int main() {
	std::cout << "=== MotionPlanner unit tests ===\n";
	test_plan_feasible_nominal();
	test_plan_rejects_zero_scale();
	test_plan_rejects_impossible_accel();
	test_start_plan_and_tick_idle();
	test_start_plan_rejects_invalid();
	test_start_plan_rejects_second_while_active();
	test_tick_covers_duration();
	test_cancel_mid_move();
	test_tick_disconnect_cancels();
	test_execute_plan_blocking();
	test_signed_distance_peak_sign();

	std::cout << "=== " << (g_failures == 0 ? "ALL PASSED" : "FAILURES")
		<< " (" << g_failures << ") ===\n";
	return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
