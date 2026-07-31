#pragma once

#include <atomic>
#include <chrono>
#include <mutex>
#include <thread>
#include "experiment/arena_config.h"
#include "motor/chase_policy.h"
#include "motor/hunt_event.h"
#include "motor/motion_planner.h"
#include "motor/prey_motor.h"

class SessionRecorder;

// TrackingFrame → hunt event / creep → MotionPlanner tick or velocity @ 50 Hz.
class ChaseController {
public:
	ChaseController(PreyMotor& motor, MotionPlanner& planner,
		const ChasePolicyConfig& cfg, int flee_direction_sign = 1,
		SessionRecorder* recorder = nullptr);
	~ChaseController();

	void start();
	void stop();
	bool is_running() const { return running_.load(); }

	// Thread-safe: called from camera callback with latest frame.
	void submit_frame(const TrackingFrame& frame);

private:
	void control_loop();

	PreyMotor& motor_;
	MotionPlanner& planner_;
	ChasePolicyConfig cfg_;
	int flee_direction_sign_;
	SessionRecorder* recorder_ = nullptr;
	HuntArmState hunt_arm_;
	// Wall time when current flee was armed (for flee_complete duration_us).
	std::chrono::steady_clock::time_point flee_start_{};
	bool flee_timing_active_ = false;
	float active_flee_mm_ = 0.0f;
	float active_threat_ = 0.0f;
	float active_distance_mm_ = 0.0f;
	float active_closing_mm_s_ = 0.0f;
	std::atomic<bool> running_{false};
	std::atomic<bool> has_frame_{false};
	std::mutex frame_mutex_;
	std::thread thread_;
	TrackingFrame latest_frame_;
};
