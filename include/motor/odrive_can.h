#pragma once

#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>

// Low-level ODrive S1 CANSimple client over Linux SocketCAN.
// Wraps all CAN frame encode/decode so PreyMotor stays chain-centric.
struct ODriveCanConfig {
	std::string interface = "can0";
	uint8_t node_id = 0; // match .axis.config.can.node_id (lab drive is 62)
	int rx_timeout_ms = 200;
};

class ODriveCan {
public:
	explicit ODriveCan(const ODriveCanConfig& cfg);

	bool open();
	void close();
	bool is_open() const { return socket_fd_ >= 0; }

	// Listen for cyclic encoder estimates; returns false on timeout or bus error.
	// const: I/O over socket_fd_ does not change logical driver state.
	// timeout_ms < 0 → cfg_.rx_timeout_ms; 0 → non-blocking poll.
	bool get_encoder_estimates(float& pos_turns, float& vel_turns_s,
		int timeout_ms = -1) const;
	bool set_input_velocity(float turns_s, float torque_ff = 0.0f);
	bool set_limits(float velocity_limit_turns_s, float current_limit_a);
	// Set_Vel_Gains (0x01b) — runtime override, not persisted to the ODrive's
	// flash config (odrivetool's save_configuration() does that separately).
	bool set_vel_gains(float vel_gain, float vel_integrator_gain);
	bool send_estop();
	bool clear_errors();
	bool check_heartbeat() const;
	// Cyclic if enabled in ODrive CAN config; timeout_ms=0 = non-blocking peek.
	bool get_iq(float& iq_setpoint, float& iq_measured, int timeout_ms = 0) const;
	// Active_Errors + Disarm_Reason (0x003) — often explains "state=8 but no motion".
	bool get_active_errors(uint32_t& active_errors, uint32_t& disarm_reason,
		int timeout_ms = 0) const;
	// ODrive autobaud stays silent until it sees host traffic — beacon then wait
	// for cyclic Heartbeat (returns false if still quiet after timeout_ms).
	bool wake_autobaud(int timeout_ms = 3000);

	// CANSimple setup — used by ODriveCalibrator during one-time setup.
	bool set_axis_state(uint32_t requested_state);
	bool set_controller_mode(uint32_t control_mode, uint32_t input_mode = 1);
	bool get_axis_state(uint32_t& axis_error, uint32_t& axis_state) const;
	bool get_heartbeat(uint32_t& axis_error, uint32_t& axis_state,
		uint8_t& procedure_result) const;
	// Motor + encoder calibration (axis will move). Blocks until IDLE or timeout.
	bool run_full_calibration(int timeout_ms = 60000);
	bool enter_velocity_mode(int timeout_ms = 10000);
	// Re-send VELOCITY+PASSTHROUGH and Set_Limits without leaving closed loop.
	bool refresh_velocity_limits();

	uint8_t node_id() const { return cfg_.node_id; }

private:
	ODriveCanConfig cfg_;
	int socket_fd_ = -1;
	// PreyMotor's status polling and ChaseController's control loop both call
	// into this instance from different threads — every public method that
	// touches socket_fd_ takes this lock. Recursive because enter_velocity_mode()
	// calls other public (self-locking) methods.
	mutable std::recursive_mutex io_mutex_;
	// Demux cache for recv_frame(): the socket has one shared RX queue but
	// several independent pollers (get_encoder_estimates, get_iq,
	// get_active_errors, heartbeat) each want a different cmd_id. Without
	// this, whichever poller drains the socket first silently discards
	// frames the others were waiting for — see recv_frame().
	mutable std::unordered_map<uint16_t, std::array<uint8_t, 8>> rx_cache_;

	bool send_frame(uint16_t cmd_id, const void* data, uint8_t len) const;
	bool send_raw_frame(uint32_t can_id_11, const void* data, uint8_t len) const;
	// On-demand request for a Get_* message that may not be cyclically
	// broadcast (Get_Iq, Get_Error) — see get_iq()/get_active_errors().
	bool send_rtr_frame(uint16_t cmd_id) const;
	bool recv_frame(uint16_t expected_cmd_id, void* data_out, uint8_t len_out,
		int timeout_ms = -1) const;
	bool wait_for_axis_state(uint32_t wanted, int timeout_ms) const;
	void flush_rx() const;
	static uint16_t can_id(uint16_t cmd_id, uint8_t node_id);
};
