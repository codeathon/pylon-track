#include "motor/trap_door_motor.h"

#include <chrono>
#include <thread>
#include "log/logger.h"

TrapDoorMotor::TrapDoorMotor(const TrapDoorConfig& cfg)
	: cfg_(cfg)
	, labjack_(cfg.labjack)
{
	use_labjack_ = (cfg.backend == "labjack");
}

bool TrapDoorMotor::connect() {
	if (!use_labjack_) {
		connected_ = true;
		status_.connected = true;
		log_info("motor", "TrapDoorMotor noop backend (manual trap operation)");
		return true;
	}
	connected_ = labjack_.open();
	status_.connected = connected_;
	if (connected_) {
		close_trap();
	}
	return connected_;
}

bool TrapDoorMotor::drive_line(bool open) {
	trap_open_ = open;
	if (!use_labjack_) {
		log_info("motor", open ? "Trap door OPEN (noop)" : "Trap door CLOSE (noop)");
		return true;
	}
	return labjack_.write_dio(open ? 1.0 : 0.0);
}

bool TrapDoorMotor::open_trap() {
	if (!connected_) {
		return false;
	}
	if (!drive_line(true)) {
		return false;
	}
	log_info("motor", "Trap door opened");
	if (cfg_.open_hold_ms > 0) {
		std::this_thread::sleep_for(std::chrono::milliseconds(cfg_.open_hold_ms));
		return close_trap();
	}
	return true;
}

bool TrapDoorMotor::close_trap() {
	if (!connected_) {
		return false;
	}
	if (!drive_line(false)) {
		return false;
	}
	log_info("motor", "Trap door closed");
	return true;
}

void TrapDoorMotor::apply(const MotorCommand& cmd) {
	if (!connected_) {
		return;
	}
	if (cmd.estop || cmd.mode == MotorMode::Estop) {
		estop();
		return;
	}
	if (cmd.mode == MotorMode::Idle) {
		stop();
		return;
	}
	if (cmd.mode == MotorMode::Velocity && cmd.velocity_mps > 0.0f) {
		open_trap();
		return;
	}
	if (cmd.mode == MotorMode::PositionDelta && cmd.position_delta_mm > 0.0f) {
		open_trap();
		return;
	}
	stop();
}

void TrapDoorMotor::stop() {
	close_trap();
}

void TrapDoorMotor::estop() {
	close_trap();
}

MotorStatus TrapDoorMotor::status() const {
	status_.connected = connected_;
	status_.heartbeat_ok = connected_;
	status_.chain_velocity_mps = trap_open_ ? 1.0f : 0.0f;
	return status_;
}
