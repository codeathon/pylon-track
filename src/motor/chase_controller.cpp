#include "motor/chase_controller.h"

#include <chrono>
#include <mutex>
#include "log/logger.h"

namespace {

constexpr int kControlPeriodMs = 20; // 50 Hz

} // namespace

ChaseController::ChaseController(PreyMotor& motor, MotionPlanner& planner,
	const ChasePolicyConfig& cfg, int flee_direction_sign)
	: motor_(motor)
	, planner_(planner)
	, cfg_(cfg)
	, flee_direction_sign_(flee_direction_sign)
{
}

ChaseController::~ChaseController() {
	stop();
}

void ChaseController::start() {
	if (running_.exchange(true)) {
		return;
	}
	// Why: planned flees and creep both command Set_Input_Vel in closed loop.
	if (!motor_.enter_velocity_mode()) {
		log_error("motor", "ChaseController: enter_velocity_mode failed");
		running_.store(false);
		return;
	}
	thread_ = std::thread(&ChaseController::control_loop, this);
	log_info("motor", "ChaseController started");
}

void ChaseController::stop() {
	if (!running_.exchange(false)) {
		return;
	}
	if (thread_.joinable()) {
		thread_.join();
	}
	// Why: abandon mid-flee — cancel then one tick so planner returns Cancelled
	// and clears active_ (cancel is cooperative; no more control-loop ticks).
	planner_.cancel();
	if (planner_.is_move_active()) {
		planner_.tick(motor_);
	}
	motor_.stop();
	log_info("motor", "ChaseController stopped");
}

void ChaseController::submit_frame(const TrackingFrame& frame) {
	std::lock_guard<std::mutex> lock(frame_mutex_);
	latest_frame_ = frame;
	has_frame_.store(true);
}

void ChaseController::control_loop() {
	while (running_.load()) {
		TrackingFrame frame;
		{
			std::lock_guard<std::mutex> lock(frame_mutex_);
			if (!has_frame_.load()) {
				std::this_thread::sleep_for(std::chrono::milliseconds(kControlPeriodMs));
				continue;
			}
			frame = latest_frame_;
		}

		// No preempt: finish current flee before accepting a new plan or creep.
		if (planner_.is_move_active()) {
			planner_.tick(motor_);
			std::this_thread::sleep_for(std::chrono::milliseconds(kControlPeriodMs));
			continue;
		}

		const ChaseDecision decision = compute_chase_decision(
			frame, cfg_, flee_direction_sign_, motor_.mm_per_turn());

		if (decision.use_planned_flee && decision.planned_flee.feasible) {
			if (!planner_.start_plan(decision.planned_flee)) {
				// Fall back to continuous creep if arming fails.
				const MotorCommand cmd = chase_decision_to_prey_command(decision);
				motor_.apply(cmd);
			} else {
				planner_.tick(motor_);
			}
		} else {
			const MotorCommand cmd = chase_decision_to_prey_command(decision);
			motor_.apply(cmd);
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(kControlPeriodMs));
	}
}
