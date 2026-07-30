#include "motor/shuttle_motor.h"

#include <chrono>

#include "log/logger.h"

namespace {
constexpr int kPollMs = 20; // matches ChaseController's control-loop granularity
}

ShuttleMotor::ShuttleMotor(const ShuttleMotorConfig& cfg)
	: cfg_(cfg)
	, labjack_(cfg.labjack)
{
	use_labjack_ = (cfg.backend == "labjack");
}

ShuttleMotor::~ShuttleMotor() {
	stop();
}

bool ShuttleMotor::connect() {
	if (!use_labjack_) {
		connected_ = true;
		log_info("motor", "ShuttleMotor noop backend (no LabJack output)");
		return true;
	}
	connected_ = labjack_.open();
	return connected_;
}

void ShuttleMotor::drive(int dir) {
	if (!use_labjack_) {
		return;
	}
	labjack_.drive(dir);
}

void ShuttleMotor::start() {
	if (running_.exchange(true)) {
		return;
	}
	thread_ = std::thread(&ShuttleMotor::run, this);
	log_info("motor", "ShuttleMotor started (wobble + hallway-end pulses)");
}

void ShuttleMotor::stop() {
	if (!running_.exchange(false)) {
		return;
	}
	if (thread_.joinable()) {
		thread_.join();
	}
	drive(0);
	log_info("motor", "ShuttleMotor stopped");
}

void ShuttleMotor::submit_position(float position_turns) {
	last_position_.store(position_turns);
	have_position_.store(true);
}

void ShuttleMotor::run() {
	// Sleep in small chunks so stop() and pulse requests stay responsive.
	auto sleep_while_running = [this](int ms) {
		const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(ms);
		while (running_.load() && std::chrono::steady_clock::now() < deadline) {
			std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
		}
	};

	int wobble_dir = 1;
	while (running_.load()) {
		if (have_position_.load()) {
			const float pos = last_position_.load();
			if (prev_position_valid_) {
				if (prev_position_ < cfg_.hallway_high_turns && pos >= cfg_.hallway_high_turns) {
					pending_pulse_dir_.store(1);
				} else if (prev_position_ > cfg_.hallway_low_turns && pos <= cfg_.hallway_low_turns) {
					pending_pulse_dir_.store(-1);
				}
			}
			prev_position_ = pos;
			prev_position_valid_ = true;
		}

		const int pulse_dir = pending_pulse_dir_.exchange(0);
		if (pulse_dir != 0) {
			log_info("motor", pulse_dir > 0
				? "Shuttle: hallway end reached — pulse forward"
				: "Shuttle: hallway start reached — pulse reverse");
			drive(pulse_dir);
			sleep_while_running(cfg_.end_pulse_ms);
			drive(0);
			wobble_dir = 1;
			continue;
		}

		drive(wobble_dir);
		sleep_while_running(cfg_.wobble_leg_ms);
		wobble_dir = -wobble_dir;
	}
	drive(0);
}
