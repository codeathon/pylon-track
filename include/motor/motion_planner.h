#pragma once

#include <atomic>
#include <cstdint>

class PreyMotor;

// High-level move API: travel a signed chain distance in a target duration.
// Runs a trapezoidal velocity profile on the calling thread at ~50 Hz.
class MotionPlanner {
public:
	~MotionPlanner();

	// Blocks until the move completes, is cancelled, or errors.
	// Returns false if motor not connected or duration/distance invalid.
	bool move_distance_mm_in_time(PreyMotor& motor, float distance_mm,
		int duration_ms, float max_accel_mps2 = 0.5f);

	void cancel();
	bool is_busy() const { return busy_.load(); }

private:
	std::atomic<bool> busy_{false};
	std::atomic<bool> cancel_{false};
};
