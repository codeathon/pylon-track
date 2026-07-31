#include "motor/chase_controller.h"

#include <chrono>
#include <mutex>
#include <string>

#include "experiment/op_timing.h"
#include "experiment/session_recorder.h"
#include "log/logger.h"

namespace {

constexpr int kControlPeriodMs = 20; // 50 Hz

int64_t steady_us_since(std::chrono::steady_clock::time_point t0) {
	return std::chrono::duration_cast<std::chrono::microseconds>(
		std::chrono::steady_clock::now() - t0).count();
}

} // namespace

ChaseController::ChaseController(PreyMotor& motor, MotionPlanner& planner,
	const ChasePolicyConfig& cfg, int flee_direction_sign, SessionRecorder* recorder)
	: motor_(motor)
	, planner_(planner)
	, cfg_(cfg)
	, flee_direction_sign_(flee_direction_sign)
	, recorder_(recorder)
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
	flee_timing_active_ = false;
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

		const int64_t now_ns = frame.host_time_ns != 0 ? frame.host_time_ns : host_now_ns();

		// No preempt: finish current flee before accepting a new plan or creep.
		if (planner_.is_move_active()) {
			const auto apply_t0 = std::chrono::steady_clock::now();
			const MoveTick tick = planner_.tick(motor_);
			record_op_duration(recorder_, "motor_apply", now_ns,
				steady_us_since(apply_t0), "flee_tick");
			if (tick == MoveTick::Done || tick == MoveTick::Cancelled) {
				const int64_t flee_us = flee_timing_active_
					? steady_us_since(flee_start_) : 0;
				record_op_duration(recorder_, "flee_complete", now_ns, flee_us,
					tick == MoveTick::Done ? "done" : "cancelled");
				if (recorder_ && recorder_->is_open()) {
					recorder_->log_chase_event(now_ns,
						tick == MoveTick::Done ? "flee_complete" : "flee_cancelled",
						active_distance_mm_, active_closing_mm_s_, active_threat_,
						active_flee_mm_, static_cast<int>(flee_us / 1000),
						0.0f, tick == MoveTick::Done ? "done" : "cancelled");
				}
				note_hunt_flee_complete(hunt_arm_, now_ns);
				flee_timing_active_ = false;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(kControlPeriodMs));
			continue;
		}

		ChaseDecision decision;
		{
			OpTimer decision_timer(recorder_, "chase_decision");
			decision = compute_chase_decision(
				frame, cfg_, flee_direction_sign_, motor_.mm_per_turn());
		}

		const bool threat_high = decision.use_planned_flee && decision.planned_flee.feasible;
		const bool hunt_event = evaluate_hunt_event(threat_high, now_ns,
			cfg_.hunt_event_min_interval_ms, hunt_arm_);

		if (hunt_event && threat_high) {
			OpTimer plan_timer(recorder_, "flee_plan");
			if (!planner_.start_plan(decision.planned_flee)) {
				plan_timer.set_detail("start_failed");
				note_hunt_flee_complete(hunt_arm_, now_ns);
				const MotorCommand cmd = chase_decision_to_prey_command(decision);
				const auto apply_t0 = std::chrono::steady_clock::now();
				motor_.apply(cmd);
				record_op_duration(recorder_, "motor_apply", now_ns,
					steady_us_since(apply_t0), "creep_fallback");
			} else {
				plan_timer.set_detail("armed");
				flee_start_ = std::chrono::steady_clock::now();
				flee_timing_active_ = true;
				active_flee_mm_ = decision.planned_flee.distance_mm;
				active_threat_ = decision.threat;
				active_distance_mm_ = frame.distance_mm;
				active_closing_mm_s_ = frame.closing_speed_mm_s;
				if (recorder_ && recorder_->is_open()) {
					recorder_->log_chase_event(now_ns, "flee_start",
						frame.distance_mm, frame.closing_speed_mm_s, decision.threat,
						decision.planned_flee.distance_mm,
						decision.planned_flee.duration_ms,
						decision.planned_flee.peak_turns_s, decision.reason);
				}
				const auto apply_t0 = std::chrono::steady_clock::now();
				planner_.tick(motor_);
				record_op_duration(recorder_, "motor_apply", now_ns,
					steady_us_since(apply_t0), "flee_start_tick");
			}
		} else {
			// Creep / chase velocity when idle (no new hunt event this tick).
			const MotorCommand cmd = chase_decision_to_prey_command(decision);
			const auto apply_t0 = std::chrono::steady_clock::now();
			motor_.apply(cmd);
			record_op_duration(recorder_, "motor_apply", now_ns,
				steady_us_since(apply_t0), decision.reason);
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(kControlPeriodMs));
	}
}
