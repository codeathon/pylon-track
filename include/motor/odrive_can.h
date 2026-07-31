#pragma once

#include <cstdint>
#include <mutex>
#include <string>

// Low-level ODrive S1 CANSimple client over Linux SocketCAN.
// Wraps all CAN frame encode/decode so PreyMotor stays chain-centric.
struct ODriveCanConfig {
	std::string interface = "can0";
	uint8_t node_id = 0;
	int rx_timeout_ms = 50;
};

class ODriveCan {
public:
	explicit ODriveCan(const ODriveCanConfig& cfg);

	bool open();
	void close();
	bool is_open() const { return socket_fd_ >= 0; }

	// Request encoder estimates; returns false on timeout or bus error.
	// const: I/O over socket_fd_ does not change logical driver state.
	bool get_encoder_estimates(float& pos_turns, float& vel_turns_s) const;
	bool set_input_velocity(float turns_s, float torque_ff = 0.0f);
	bool send_estop();
	bool check_heartbeat() const;

	// CANSimple setup — used by ODriveCalibrator during one-time setup.
	bool set_axis_state(uint32_t requested_state);
	bool set_controller_mode(int32_t control_mode, int32_t input_mode = 1);
	bool get_axis_state(uint32_t& axis_error, uint32_t& axis_state) const;
	bool enter_velocity_mode(int timeout_ms = 10000);

	uint8_t node_id() const { return cfg_.node_id; }

private:
	ODriveCanConfig cfg_;
	int socket_fd_ = -1;
	// PreyMotor's status polling and ChaseController's control loop both call
	// into this instance from different threads — every public method that
	// touches socket_fd_ takes this lock. Recursive because enter_velocity_mode()
	// calls other public (self-locking) methods.
	mutable std::recursive_mutex io_mutex_;

	bool send_frame(uint16_t cmd_id, const void* data, uint8_t len) const;
	bool recv_frame(uint16_t expected_cmd_id, void* data_out, uint8_t len_out) const;
	static uint16_t can_id(uint16_t cmd_id, uint8_t node_id);
};
