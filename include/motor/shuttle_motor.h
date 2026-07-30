#pragma once

#include <atomic>
#include <string>
#include <thread>

#include "motor/labjack_hbridge.h"

// Shuttle motor config — backend selects hardware driver.
struct ShuttleMotorConfig {
	// "noop" = software stub; "labjack" = LabJack H-bridge DIO (FIO4/FIO5).
	std::string backend = "noop";
	LabjackHBridgeConfig labjack;

	// Idle wobble: alternate direction every wobble_leg_ms, continuously —
	// simulates small back-and-forth movement.
	int wobble_leg_ms = 300;

	// When the tracked pulley position crosses hallway_high_turns going up,
	// drive forward for end_pulse_ms. When it crosses hallway_low_turns
	// coming back down, drive reverse for end_pulse_ms.
	int end_pulse_ms = 1000;
	float hallway_high_turns = 15.0f;
	float hallway_low_turns = 5.0f;
};

// Drives a DC motor over two LabJack digital lines (H-bridge): wobbles back
// and forth in place to simulate movement, and fires a longer directional
// pulse when the tracked pulley position crosses either end of the hallway.
// Runs its own background thread — feed it position updates from whatever
// is polling the drive motor (e.g. PreyMotor::read_position_turns()).
class ShuttleMotor {
public:
	explicit ShuttleMotor(const ShuttleMotorConfig& cfg);
	~ShuttleMotor();

	ShuttleMotor(const ShuttleMotor&) = delete;
	ShuttleMotor& operator=(const ShuttleMotor&) = delete;

	bool connect();
	void start();
	void stop();
	bool is_running() const { return running_.load(); }

	// Thread-safe: call from wherever the drive motor's position is polled.
	void submit_position(float position_turns);

private:
	void run();
	void drive(int dir);

	ShuttleMotorConfig cfg_;
	LabjackHBridge labjack_;
	bool use_labjack_ = false;
	bool connected_ = false;

	std::atomic<bool> running_{false};
	std::thread thread_;

	std::atomic<float> last_position_{0.0f};
	std::atomic<bool> have_position_{false};
	std::atomic<int> pending_pulse_dir_{0}; // 0 = none, +1/-1 = pulse requested

	float prev_position_ = 0.0f;
	bool prev_position_valid_ = false;
};
