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
};
