#pragma once

#include <cstdint>
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
	bool get_encoder_estimates(float& pos_turns, float& vel_turns_s);
	bool set_input_velocity(float turns_s, float torque_ff = 0.0f);
	bool send_estop();
	bool check_heartbeat();

	// CANSimple setup — used by ODriveCalibrator during one-time setup.
	bool set_axis_state(uint32_t requested_state);
	bool set_controller_mode(int32_t control_mode, int32_t input_mode = 1);
	bool get_axis_state(uint32_t& axis_error, uint32_t& axis_state);
	bool enter_velocity_mode(int timeout_ms = 10000);

	uint8_t node_id() const { return cfg_.node_id; }

private:
	ODriveCanConfig cfg_;
	int socket_fd_ = -1;

	bool send_frame(uint16_t cmd_id, const void* data, uint8_t len);
	bool recv_frame(uint16_t expected_cmd_id, void* data_out, uint8_t len_out);
	static uint16_t can_id(uint16_t cmd_id, uint8_t node_id);
};
