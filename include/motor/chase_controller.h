#pragma once

#include <atomic>
#include <mutex>
#include <thread>
#include "experiment/arena_config.h"
#include "motor/chase_policy.h"
#include "motor/prey_motor.h"

// TrackingFrame → chase policy → PreyMotor at a fixed control rate.
class ChaseController {
public:
	ChaseController(PreyMotor& motor, const ChasePolicyConfig& cfg,
		int flee_direction_sign = 1);
	~ChaseController();

	void start();
	void stop();
	bool is_running() const { return running_.load(); }

	// Thread-safe: called from camera callback with latest frame.
	void submit_frame(const TrackingFrame& frame);

private:
	void control_loop();

	PreyMotor& motor_;
	ChasePolicyConfig cfg_;
	int flee_direction_sign_;
	std::atomic<bool> running_{false};
	std::atomic<bool> has_frame_{false};
	std::mutex frame_mutex_;
	std::thread thread_;
	TrackingFrame latest_frame_;
};
