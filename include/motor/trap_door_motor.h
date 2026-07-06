#pragma once

#include <string>
#include "motor/i_motor.h"
#include "motor/labjack_io.h"

// Trap door actuator config — backend selects hardware driver.
struct TrapDoorConfig {
	// "noop" = software stub; "labjack" = LabJack digital output.
	std::string backend = "noop";
	LabJackConfig labjack;
	// Hold open line high for this duration on open command (0 = latch until close).
	int open_hold_ms = 500;
};

// Trap door motor — IMotor adapter for LabJack DIO or noop stub.
// MotorCommand mapping:
//   Velocity > 0  → open trap
//   Idle          → close trap
//   Estop         → close trap immediately
class TrapDoorMotor : public IMotor {
public:
	explicit TrapDoorMotor(const TrapDoorConfig& cfg);

	bool connect() override;
	void apply(const MotorCommand& cmd) override;
	void stop() override;
	void estop() override;
	MotorStatus status() const override;

	bool open_trap();
	bool close_trap();
	bool is_open() const { return trap_open_; }

	const TrapDoorConfig& config() const { return cfg_; }

private:
	TrapDoorConfig cfg_;
	LabJackIo labjack_;
	bool use_labjack_ = false;
	bool connected_ = false;
	bool trap_open_ = false;
	mutable MotorStatus status_;

	bool drive_line(bool open);
};
