#include "motor/odrive_can.h"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <iomanip>
#include <sstream>
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
constexpr uint16_t CMD_SET_AXIS_STATE = 0x007;
constexpr uint16_t CMD_SET_CONTROLLER_MODE = 0x00b;
constexpr uint16_t CMD_SET_INPUT_VEL = 0x00d;
constexpr uint16_t CMD_SET_LIMITS = 0x00f;
constexpr uint16_t CMD_CLEAR_ERRORS = 0x018;

constexpr uint32_t AXIS_STATE_IDLE = 1;
constexpr uint32_t AXIS_STATE_FULL_CALIBRATION_SEQUENCE = 3;
constexpr uint32_t AXIS_STATE_CLOSED_LOOP_CONTROL = 8;
constexpr uint32_t CONTROL_MODE_VELOCITY_CONTROL = 2;
constexpr uint32_t INPUT_MODE_PASSTHROUGH = 1;
// ODrive ProcedureResult / Error (subset used in setup diagnostics).
constexpr uint8_t kProcedureSuccess = 0;
constexpr uint32_t kErrCalibration = 0x40000000u;

const char* procedure_result_name(uint8_t result) {
	switch (result) {
	case 0: return "SUCCESS";
	case 1: return "BUSY";
	case 2: return "CANCELLED";
	case 3: return "DISARMED";
	case 4: return "NO_RESPONSE"; // encoder wiring/config
	case 5: return "POLE_PAIR_CPR_MISMATCH";
	case 6: return "PHASE_RESISTANCE_OUT_OF_RANGE"; // motor leads / calib voltage
	case 7: return "PHASE_INDUCTANCE_OUT_OF_RANGE";
	case 8: return "UNBALANCED_PHASES";
	case 9: return "INVALID_MOTOR_TYPE";
	case 10: return "ILLEGAL_HALL_STATE";
	case 11: return "TIMEOUT";
	default: return "UNKNOWN";
	}
}

std::string hex_u32(uint32_t value) {
	std::ostringstream oss;
	oss << "0x" << std::hex << value;
	return oss.str();
}
// Safe defaults so a GUI left at vel_limit=0 cannot silently clamp Set_Input_Vel.
constexpr float kDefaultVelLimitTurnsS = 10.0f;
constexpr float kDefaultCurrentLimitA = 40.0f;

void pack_float_le(float value, uint8_t* out) {
	std::memcpy(out, &value, sizeof(float));
}

bool unpack_float_le(const uint8_t* in, float& value) {
	std::memcpy(&value, in, sizeof(float));
	return true;
}

} // namespace

ODriveCan::ODriveCan(const ODriveCanConfig& cfg) : cfg_(cfg) {}

// ODrive CANSimple: arbitration ID = (node_id << 5) | cmd_id  (11-bit).
uint16_t ODriveCan::can_id(uint16_t cmd_id, uint8_t node_id) {
	return static_cast<uint16_t>((static_cast<uint16_t>(node_id) << 5) | (cmd_id & 0x1F));
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

bool ODriveCan::send_raw_frame(uint32_t can_id_11, const void* data, uint8_t len) const {
	if (socket_fd_ < 0) {
		return false;
	}
	can_frame frame{};
	frame.can_id = can_id_11 & CAN_SFF_MASK;
	frame.can_dlc = len;
	if (data && len > 0) {
		std::memcpy(frame.data, data, std::min<uint8_t>(len, 8));
	}
	const ssize_t n = write(socket_fd_, &frame, sizeof(frame));
	return n == static_cast<ssize_t>(sizeof(frame));
}

bool ODriveCan::send_frame(uint16_t cmd_id, const void* data, uint8_t len) const {
	return send_raw_frame(can_id(cmd_id, cfg_.node_id), data, len);
}

bool ODriveCan::wake_autobaud(int timeout_ms) {
	if (socket_fd_ < 0) {
		return false;
	}
	// Any host traffic ≥10 Hz wakes ODrive autobaud out of silent scan mode.
	flush_rx();
	if (check_heartbeat()) {
		return true;
	}

	log_info("motor", "CAN quiet — sending autobaud beacon for up to "
		+ std::to_string(timeout_ms) + " ms");
	const auto deadline = std::chrono::steady_clock::now()
		+ std::chrono::milliseconds(timeout_ms);
	// Harmless ID: our node Get_Version (empty). Docs: any ID/payload works.
	const uint32_t beacon_id = can_id(0x000, cfg_.node_id);
	while (std::chrono::steady_clock::now() < deadline) {
		send_raw_frame(beacon_id, nullptr, 0);
		// Short poll for cyclic heartbeat instead of full rx_timeout each loop.
		pollfd pfd{};
		pfd.fd = socket_fd_;
		pfd.events = POLLIN;
		if (poll(&pfd, 1, 50) > 0) {
			can_frame frame{};
			if (read(socket_fd_, &frame, sizeof(frame))
					>= static_cast<ssize_t>(sizeof(can_frame))) {
				const uint32_t id = frame.can_id & CAN_SFF_MASK;
				const uint16_t cmd = static_cast<uint16_t>(id & 0x1F);
				const uint8_t node = static_cast<uint8_t>((id >> 5) & 0x3F);
				if (node == cfg_.node_id && cmd == CMD_HEARTBEAT) {
					log_info("motor", "ODrive heartbeat after autobaud beacon");
					return true;
				}
			}
		}
	}
	return check_heartbeat();
}

bool ODriveCan::recv_frame(uint16_t expected_cmd_id, void* data_out, uint8_t len_out) const {
	if (socket_fd_ < 0) {
		return false;
	}
	pollfd pfd{};
	pfd.fd = socket_fd_;
	pfd.events = POLLIN;
	const auto deadline = std::chrono::steady_clock::now()
		+ std::chrono::milliseconds(cfg_.rx_timeout_ms);

	// Bus is busy with cyclic encoder/heartbeat — skip non-matching frames until timeout.
	while (std::chrono::steady_clock::now() < deadline) {
		const auto left = std::chrono::duration_cast<std::chrono::milliseconds>(
			deadline - std::chrono::steady_clock::now()).count();
		if (poll(&pfd, 1, static_cast<int>(std::max<int64_t>(left, 0))) <= 0) {
			return false;
		}

		can_frame frame{};
		const ssize_t n = read(socket_fd_, &frame, sizeof(frame));
		if (n < static_cast<ssize_t>(sizeof(can_frame))) {
			continue;
		}
		const uint32_t id = frame.can_id & CAN_SFF_MASK;
		const uint16_t cmd_id = static_cast<uint16_t>(id & 0x1F);
		const uint8_t node_id = static_cast<uint8_t>((id >> 5) & 0x3F);
		if (cmd_id != expected_cmd_id || node_id != cfg_.node_id) {
			continue;
		}
		if (data_out && len_out > 0) {
			const uint8_t copy_len = std::min(len_out, frame.can_dlc);
			std::memcpy(data_out, frame.data, copy_len);
		}
		return true;
	}
	return false;
}

bool ODriveCan::get_encoder_estimates(float& pos_turns, float& vel_turns_s) const {
	// Cyclic by default (~10 ms); listen rather than request.
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

bool ODriveCan::set_limits(float velocity_limit_turns_s, float current_limit_a) {
	uint8_t buf[8] = {};
	pack_float_le(velocity_limit_turns_s, buf);
	pack_float_le(current_limit_a, buf + 4);
	return send_frame(CMD_SET_LIMITS, buf, sizeof(buf));
}

bool ODriveCan::send_estop() {
	return send_frame(CMD_ESTOP, nullptr, 0);
}

bool ODriveCan::clear_errors() {
	// Identify=0: clear only (do not blink status LED).
	const uint8_t identify = 0;
	return send_frame(CMD_CLEAR_ERRORS, &identify, 1);
}

bool ODriveCan::check_heartbeat() const {
	// Heartbeat is cyclic (~100 ms) from the drive — do not TX a fake request.
	uint8_t buf[8] = {};
	return recv_frame(CMD_HEARTBEAT, buf, sizeof(buf));
}

bool ODriveCan::set_axis_state(uint32_t requested_state) {
	uint8_t buf[8] = {};
	std::memcpy(buf, &requested_state, sizeof(uint32_t));
	// Pad to 8 bytes — Host→ODrive reserved fields must be zero.
	return send_frame(CMD_SET_AXIS_STATE, buf, sizeof(buf));
}

bool ODriveCan::set_controller_mode(uint32_t control_mode, uint32_t input_mode) {
	uint8_t buf[8] = {};
	std::memcpy(buf, &control_mode, sizeof(uint32_t));
	std::memcpy(buf + 4, &input_mode, sizeof(uint32_t));
	return send_frame(CMD_SET_CONTROLLER_MODE, buf, sizeof(buf));
}

bool ODriveCan::get_heartbeat(uint32_t& axis_error, uint32_t& axis_state,
	uint8_t& procedure_result) const
{
	uint8_t buf[8] = {};
	if (!recv_frame(CMD_HEARTBEAT, buf, sizeof(buf))) {
		return false;
	}
	std::memcpy(&axis_error, buf, sizeof(uint32_t));
	// Heartbeat: Axis_Error u32, Axis_State u8, Procedure_Result u8, ...
	axis_state = buf[4];
	procedure_result = buf[5];
	return true;
}

bool ODriveCan::get_axis_state(uint32_t& axis_error, uint32_t& axis_state) const {
	uint8_t procedure_result = 0;
	return get_heartbeat(axis_error, axis_state, procedure_result);
}

bool ODriveCan::run_full_calibration(int timeout_ms) {
	clear_errors();
	if (!set_axis_state(AXIS_STATE_IDLE)) {
		log_error("motor", "Set axis IDLE failed before calibration");
		return false;
	}
	flush_rx();
	wait_for_axis_state(AXIS_STATE_IDLE, 2000);

	if (!set_axis_state(AXIS_STATE_FULL_CALIBRATION_SEQUENCE)) {
		log_error("motor", "Set_Axis_State FULL_CALIBRATION failed");
		return false;
	}
	log_info("motor", "ODrive full calibration started (motor will move)");

	const auto deadline = std::chrono::steady_clock::now()
		+ std::chrono::milliseconds(timeout_ms);
	bool saw_busy_state = false;
	while (std::chrono::steady_clock::now() < deadline) {
		uint32_t err = 0;
		uint32_t state = 0;
		uint8_t result = 0;
		if (!get_heartbeat(err, state, result)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			continue;
		}
		if (state != AXIS_STATE_IDLE) {
			saw_busy_state = true;
		}
		// Done when back in IDLE after leaving it for the calibration sequence.
		if (saw_busy_state && state == AXIS_STATE_IDLE) {
			if (err != 0 || result != kProcedureSuccess) {
				std::string msg = "Calibration failed: err=" + hex_u32(err)
					+ " procedure_result=" + std::to_string(result)
					+ " (" + procedure_result_name(result) + ")";
				if (err & kErrCalibration) {
					msg += " [CALIBRATION_ERROR — check motor phase wiring, "
						"calibration_current, resistance_calib_max_voltage; "
						"use ODrive GUI/odrivetool for details]";
				}
				log_error("motor", msg);
				return false;
			}
			log_info("motor", "ODrive full calibration succeeded");
			return true;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
	log_error("motor", "Timed out waiting for full calibration");
	return false;
}

bool ODriveCan::wait_for_axis_state(uint32_t wanted, int timeout_ms) const {
	const auto deadline = std::chrono::steady_clock::now()
		+ std::chrono::milliseconds(timeout_ms);
	while (std::chrono::steady_clock::now() < deadline) {
		uint32_t err = 0;
		uint32_t state = 0;
		if (get_axis_state(err, state) && state == wanted) {
			return true;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
	return false;
}

void ODriveCan::flush_rx() const {
	if (socket_fd_ < 0) {
		return;
	}
	// Drop buffered cyclic frames so the next heartbeat reflects post-command state.
	pollfd pfd{};
	pfd.fd = socket_fd_;
	pfd.events = POLLIN;
	while (poll(&pfd, 1, 0) > 0) {
		can_frame frame{};
		if (read(socket_fd_, &frame, sizeof(frame)) <= 0) {
			break;
		}
	}
}

bool ODriveCan::enter_velocity_mode(int timeout_ms) {
	// Position-hold from a prior run looks like state=8 with Set_Input_Vel ignored.
	// Force IDLE, apply velocity mode + limits, then re-enter closed loop.
	clear_errors();
	if (!set_axis_state(AXIS_STATE_IDLE)) {
		log_error("motor", "Set axis IDLE failed");
		return false;
	}
	flush_rx();
	if (!wait_for_axis_state(AXIS_STATE_IDLE, std::min(timeout_ms, 2000))) {
		log_error("motor", "Timed out waiting for IDLE before velocity mode");
		return false;
	}

	if (!set_controller_mode(CONTROL_MODE_VELOCITY_CONTROL, INPUT_MODE_PASSTHROUGH)) {
		log_error("motor", "Set controller mode failed");
		return false;
	}
	if (!set_limits(kDefaultVelLimitTurnsS, kDefaultCurrentLimitA)) {
		log_error("motor", "Set_Limits failed");
		return false;
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(50));

	if (!set_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL)) {
		log_error("motor", "Set axis state failed");
		return false;
	}
	flush_rx();
	if (!wait_for_axis_state(AXIS_STATE_CLOSED_LOOP_CONTROL, timeout_ms)) {
		uint32_t err = 0;
		uint32_t state = 0;
		if (get_axis_state(err, state)) {
			log_error("motor", "Timed out waiting for closed-loop control (state="
				+ std::to_string(state) + " err=0x" + std::to_string(err) + ")");
		} else {
			log_error("motor", "Timed out waiting for closed-loop control");
		}
		return false;
	}

	log_info("motor", "ODrive axis in closed-loop velocity mode");
	return true;
}
