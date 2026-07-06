#include "motor/odrive_can.h"

#include <chrono>
#include <cstring>
#include <poll.h>
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

	log_info("motor", "CAN open on " + cfg_.interface + " node " + std::to_string(cfg_.node_id));
	return true;
}

void ODriveCan::close() {
	if (socket_fd_ >= 0) {
		::close(socket_fd_);
		socket_fd_ = -1;
	}
}

bool ODriveCan::send_frame(uint16_t cmd_id, const void* data, uint8_t len) {
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

bool ODriveCan::recv_frame(uint16_t expected_cmd_id, void* data_out, uint8_t len_out) {
	if (socket_fd_ < 0) {
		return false;
	}
	pollfd pfd{};
	pfd.fd = socket_fd_;
	pfd.events = POLLIN;
	const int poll_ms = cfg_.rx_timeout_ms;
	if (poll(&pfd, 1, poll_ms) <= 0) {
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
		return false;
	}
	if (data_out && len_out > 0) {
		const uint8_t copy_len = std::min(len_out, frame.can_dlc);
		std::memcpy(data_out, frame.data, copy_len);
	}
	return true;
}

bool ODriveCan::get_encoder_estimates(float& pos_turns, float& vel_turns_s) {
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
	uint8_t buf[8] = {};
	pack_float_le(turns_s, buf);
	pack_float_le(torque_ff, buf + 4);
	return send_frame(CMD_SET_INPUT_VEL, buf, sizeof(buf));
}

bool ODriveCan::send_estop() {
	return send_frame(CMD_ESTOP, nullptr, 0);
}

bool ODriveCan::check_heartbeat() {
	if (!send_frame(CMD_HEARTBEAT, nullptr, 0)) {
		return false;
	}
	uint8_t buf[8] = {};
	return recv_frame(CMD_HEARTBEAT, buf, sizeof(buf));
}
