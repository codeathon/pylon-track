#include "motor/chase_controller.h"

#include <chrono>
#include <mutex>
#include "log/logger.h"

namespace {

constexpr int kControlPeriodMs = 20; // 50 Hz

} // namespace

ChaseController::ChaseController(PreyMotor& motor, const ChasePolicyConfig& cfg,
	int flee_direction_sign)
	: motor_(motor)
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
		const ChaseDecision decision = compute_chase_decision(
			frame, cfg_, flee_direction_sign_);
		const MotorCommand cmd = chase_decision_to_prey_command(decision);
		motor_.apply(cmd);
		std::this_thread::sleep_for(std::chrono::milliseconds(kControlPeriodMs));
	}
}
