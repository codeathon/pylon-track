#pragma once

#include "motor/motor_types.h"

// Common contract for arena motors driven by MotorCommand (prey chain).
class IMotor {
public:
	virtual ~IMotor() = default;

	virtual bool connect() = 0;
	virtual void apply(const MotorCommand& cmd) = 0;
	virtual void stop() = 0;
	virtual void estop() = 0;
	virtual MotorStatus status() const = 0;
	// Why: MotionPlanner::tick must not call status() — on PreyMotor that blocks
	// on CAN encoder RX (up to ~200 ms) and short distance moves expire before
	// any Set_Input_Vel is sent. Cached connect flag only; no bus I/O.
	virtual bool is_connected() const { return status().connected; }
	// Direct Set_Input_Vel (motor turns/s). Default unsupported — callers fall
	// back to apply(velocity_mps). PreyMotor overrides to the same path as
	// test_odrive_move --vel-turns-s (known-good on the lab rig).
	virtual bool command_turns_s(float /*turns_s*/) { return false; }
};
