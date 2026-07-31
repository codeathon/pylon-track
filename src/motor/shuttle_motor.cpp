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

void ShuttleMotor::check_crossing() {
	if (!have_position_.load()) {
		return;
	}
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

void ShuttleMotor::run() {
	// Position is checked every kPollMs tick regardless of what the wobble
	// is doing, so a fast chase-driven crossing can't land inside one long
	// blocking sleep and go unnoticed — only an active pulse (which is
	// itself the reaction to a crossing) isn't preempted by a new one.
	int wobble_dir = 1;
	auto wobble_deadline = std::chrono::steady_clock::now()
		+ std::chrono::milliseconds(cfg_.wobble_leg_ms);
	drive(wobble_dir);

	while (running_.load()) {
		check_crossing();

		const int pulse_dir = pending_pulse_dir_.exchange(0);
		if (pulse_dir != 0) {
			log_info("motor", pulse_dir > 0
				? "Shuttle: hallway end reached — pulse forward"
				: "Shuttle: hallway start reached — pulse reverse");
			drive(pulse_dir);
			const auto pulse_deadline = std::chrono::steady_clock::now()
				+ std::chrono::milliseconds(cfg_.end_pulse_ms);
			while (running_.load() && std::chrono::steady_clock::now() < pulse_deadline) {
				std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
			}
			wobble_dir = 1;
			wobble_deadline = std::chrono::steady_clock::now()
				+ std::chrono::milliseconds(cfg_.wobble_leg_ms);
			drive(wobble_dir);
			continue;
		}

		if (std::chrono::steady_clock::now() >= wobble_deadline) {
			wobble_dir = -wobble_dir;
			drive(wobble_dir);
			wobble_deadline = std::chrono::steady_clock::now()
				+ std::chrono::milliseconds(cfg_.wobble_leg_ms);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(kPollMs));
	}
	drive(0);
}
