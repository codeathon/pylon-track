#include "motor/odrive_can.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <poll.h>
#include <thread>
#include <unistd.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include "log/logger.h"

namespace {

// ODrive CANSimple command IDs (see ODrive CAN protocol docs).
constexpr uint16_t CMD_HEARTBEAT = 0x001;
constexpr uint16_t CMD_ESTOP = 0x002;
constexpr uint16_t CMD_GET_ENCODER_ESTIMATES = 0x009;
constexpr uint16_t CMD_SET_INPUT_VEL = 0x00d;
constexpr uint16_t CMD_SET_AXIS_STATE = 0x007;
constexpr uint16_t CMD_SET_CONTROLLER_MODE = 0x00b;

constexpr uint32_t AXIS_STATE_CLOSED_LOOP_CONTROL = 8;
constexpr int32_t CONTROL_MODE_VELOCITY_CONTROL = 2;
constexpr int32_t INPUT_MODE_PASSTHROUGH = 1;

void pack_float_le(float value, uint8_t* out) {
	std::memcpy(out, &value, sizeof(float));
}

bool unpack_float_le(const uint8_t* in, float& value) {
	std::memcpy(&value, in, sizeof(float));
	return true;
}

} // namespace

ODriveCan::ODriveCan(const ODriveCanConfig& cfg) : cfg_(cfg) {}

uint16_t ODriveCan::can_id(uint16_t cmd_id, uint8_t node_id) {
	return static_cast<uint16_t>((cmd_id << 5) | node_id);
}

bool ODriveCan::open() {
	std::lock_guard<std::recursive_mutex> lock(io_mutex_);
	if (socket_fd_ >= 0) {
		return true;
	}
	socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (socket_fd_ < 0) {
		log_error("motor", "CAN socket create failed on " + cfg_.interface);
		return false;
	}

	ifreq ifr{};
	std::strncpy(ifr.ifr_name, cfg_.interface.c_str(), IFNAMSIZ - 1);
	if (ioctl(socket_fd_, SIOCGIFINDEX, &ifr) < 0) {
		log_error("motor", "CAN interface not found: " + cfg_.interface);
		close();
		return false;
	}

	sockaddr_can addr{};
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;
	if (bind(socket_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
		log_error("motor", "CAN bind failed on " + cfg_.interface);
		close();
		return false;
	}

	// Restrict the socket to frames addressed to this node (any cmd_id) —
	// without this every frame on the bus (other axes' heartbeats, etc.) is
	// delivered here too, which recv_frame would otherwise have to wade through.
	can_filter filter{};
	filter.can_id = cfg_.node_id;
	filter.can_mask = 0x1F;
	if (setsockopt(socket_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) < 0) {
		log_error("motor", "CAN filter setup failed on " + cfg_.interface + " (continuing unfiltered)");
	}

	log_info("motor", "CAN open on " + cfg_.interface + " node " + std::to_string(cfg_.node_id));
	return true;
}

void ODriveCan::close() {
	std::lock_guard<std::recursive_mutex> lock(io_mutex_);
	if (socket_fd_ >= 0) {
		::close(socket_fd_);
		socket_fd_ = -1;
	}
}

bool ODriveCan::send_frame(uint16_t cmd_id, const void* data, uint8_t len) const {
	if (socket_fd_ < 0) {
		return false;
	}
	can_frame frame{};
	frame.can_id = can_id(cmd_id, cfg_.node_id);
	frame.can_dlc = len;
	if (data && len > 0) {
		std::memcpy(frame.data, data, len);
	}
	const ssize_t n = write(socket_fd_, &frame, sizeof(frame));
	return n == static_cast<ssize_t>(sizeof(frame));
}

bool ODriveCan::recv_frame(uint16_t expected_cmd_id, void* data_out, uint8_t len_out) const {
	if (socket_fd_ < 0) {
		return false;
	}
	// Drain frames until one matches or the timeout elapses — a single
	// non-matching frame (e.g. a periodic heartbeat queued just ahead of the
	// actual response) must not make the caller give up early.
	const auto deadline = std::chrono::steady_clock::now()
		+ std::chrono::milliseconds(cfg_.rx_timeout_ms);
	while (true) {
		const auto now = std::chrono::steady_clock::now();
		if (now >= deadline) {
			return false;
		}
		const int remaining_ms = static_cast<int>(
			std::chrono::duration_cast<std::chrono::milliseconds>(deadline - now).count());
		pollfd pfd{};
		pfd.fd = socket_fd_;
		pfd.events = POLLIN;
		if (poll(&pfd, 1, remaining_ms) <= 0) {
			return false;
		}

		can_frame frame{};
		const ssize_t n = read(socket_fd_, &frame, sizeof(frame));
		if (n < static_cast<ssize_t>(sizeof(can_frame))) {
			return false;
		}
		const uint16_t cmd_id = static_cast<uint16_t>((frame.can_id >> 5) & 0x7FF);
		const uint8_t node_id = static_cast<uint8_t>(frame.can_id & 0x1F);
		if (cmd_id != expected_cmd_id || node_id != cfg_.node_id) {
			continue;
		}
		if (data_out && len_out > 0) {
			const uint8_t copy_len = std::min(len_out, frame.can_dlc);
			std::memcpy(data_out, frame.data, copy_len);
		}
		return true;
	}
}

bool ODriveCan::get_encoder_estimates(float& pos_turns, float& vel_turns_s) const {
	std::lock_guard<std::recursive_mutex> lock(io_mutex_);
	if (!send_frame(CMD_GET_ENCODER_ESTIMATES, nullptr, 0)) {
		return false;
	}
	uint8_t buf[8] = {};
	if (!recv_frame(CMD_GET_ENCODER_ESTIMATES, buf, sizeof(buf))) {
		return false;
	}
	unpack_float_le(buf, pos_turns);
	unpack_float_le(buf + 4, vel_turns_s);
	return true;
}

bool ODriveCan::set_input_velocity(float turns_s, float torque_ff) {
	std::lock_guard<std::recursive_mutex> lock(io_mutex_);
	uint8_t buf[8] = {};
	pack_float_le(turns_s, buf);
	pack_float_le(torque_ff, buf + 4);
	return send_frame(CMD_SET_INPUT_VEL, buf, sizeof(buf));
}

bool ODriveCan::send_estop() {
	std::lock_guard<std::recursive_mutex> lock(io_mutex_);
	return send_frame(CMD_ESTOP, nullptr, 0);
}

bool ODriveCan::check_heartbeat() const {
	std::lock_guard<std::recursive_mutex> lock(io_mutex_);
	if (!send_frame(CMD_HEARTBEAT, nullptr, 0)) {
		return false;
	}
	uint8_t buf[8] = {};
	return recv_frame(CMD_HEARTBEAT, buf, sizeof(buf));
}

bool ODriveCan::set_axis_state(uint32_t requested_state) {
	std::lock_guard<std::recursive_mutex> lock(io_mutex_);
	uint8_t buf[8] = {};
	std::memcpy(buf, &requested_state, sizeof(uint32_t));
	return send_frame(CMD_SET_AXIS_STATE, buf, sizeof(uint32_t));
}

bool ODriveCan::set_controller_mode(int32_t control_mode, int32_t input_mode) {
	std::lock_guard<std::recursive_mutex> lock(io_mutex_);
	uint8_t buf[8] = {};
	std::memcpy(buf, &control_mode, sizeof(int32_t));
	std::memcpy(buf + 4, &input_mode, sizeof(int32_t));
	return send_frame(CMD_SET_CONTROLLER_MODE, buf, sizeof(buf));
}

bool ODriveCan::get_axis_state(uint32_t& axis_error, uint32_t& axis_state) const {
	std::lock_guard<std::recursive_mutex> lock(io_mutex_);
	if (!send_frame(CMD_HEARTBEAT, nullptr, 0)) {
		return false;
	}
	uint8_t buf[8] = {};
	if (!recv_frame(CMD_HEARTBEAT, buf, sizeof(buf))) {
		return false;
	}
	// CANSimple Heartbeat layout: Axis_Error is the full 4-byte word at
	// offset 0, but Axis_State is a single byte at offset 4 — bytes 5-7 are
	// Procedure_result/Trajectory_done_flag, not part of the state value.
	// Reading 4 bytes there previously pulled those in as garbage high bits.
	std::memcpy(&axis_error, buf, sizeof(uint32_t));
	axis_state = static_cast<uint32_t>(buf[4]);
	return true;
}

bool ODriveCan::enter_velocity_mode(int timeout_ms) {
	std::lock_guard<std::recursive_mutex> lock(io_mutex_);
	if (!set_controller_mode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH)) {
		log_error("motor", "Set controller mode failed");
		return false;
	}
	if (!set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL)) {
		log_error("motor", "Set axis state failed");
		return false;
	}
	const auto deadline = std::chrono::steady_clock::now()
		+ std::chrono::milliseconds(timeout_ms);
	while (std::chrono::steady_clock::now() < deadline) {
		uint32_t err = 0;
		uint32_t state = 0;
		if (get_axis_state(err, state) && state == AXIS_STATE_CLOSED_LOOP_CONTROL) {
			log_info("motor", "ODrive axis in closed-loop velocity mode");
			return true;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	log_error("motor", "Timed out waiting for closed-loop control");
	return false;
}
