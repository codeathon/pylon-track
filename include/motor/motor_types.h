#pragma once

#include <cstdint>

// Hardware-neutral motor command envelope consumed by IMotor drivers.
enum class MotorMode : uint8_t {
	Idle,
	Velocity,
	PositionDelta,
	Estop
};

struct MotorStatus {
	bool connected = false;
	bool heartbeat_ok = false;
	float position_turns = 0.0f;
	float velocity_turns_s = 0.0f;
	float chain_position_mm = 0.0f;
	float chain_velocity_mps = 0.0f;
};

struct MotorCommand {
	int64_t command_time_ns = 0;
	MotorMode mode = MotorMode::Idle;
	float velocity_mps = 0.0f;        // chain linear m/s (signed)
	float position_delta_mm = 0.0f;   // incremental move (future)
	float max_accel_mps2 = 0.0f;      // 0 = driver default
	bool estop = false;
};
