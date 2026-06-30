#include "motor/odrive.h"
#include "log/logger.h"

#include <algorithm>
#include <cerrno>
#include <cstring>
#include <stdexcept>
#include <string>

// POSIX serial — Linux only (matches the rest of the project).
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <poll.h>

// ── ODrive ASCII protocol constants ──────────────────────────────────────────
// Axis states
static constexpr int AXIS_STATE_IDLE               = 1;
static constexpr int AXIS_STATE_CLOSED_LOOP_CONTROL = 8;
// Control modes
static constexpr int CONTROL_MODE_VELOCITY         = 2;

// ── Helpers ──────────────────────────────────────────────────────────────────

static speed_t baud_constant(int baud) {
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default:     return B115200;
    }
}

// ── ODrive implementation ─────────────────────────────────────────────────────

bool ODrive::open(const std::string& port, int baud) {
    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        LOG_INFO("ODrive: cannot open " + port + " — " + std::strerror(errno));
        return false;
    }

    termios tty{};
    if (tcgetattr(fd_, &tty) != 0) {
        LOG_INFO("ODrive: tcgetattr failed — " + std::string(std::strerror(errno)));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    const speed_t spd = baud_constant(baud);
    cfsetispeed(&tty, spd);
    cfsetospeed(&tty, spd);

    // 8N1, no flow control, raw mode
    cfmakeraw(&tty);
    tty.c_cflag |=  (CLOCAL | CREAD);
    tty.c_cflag &= ~CRTSCTS;

    // Blocking read with 200 ms timeout
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 2;   // 200 ms in tenths of a second

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
        LOG_INFO("ODrive: tcsetattr failed — " + std::string(std::strerror(errno)));
        ::close(fd_);
        fd_ = -1;
        return false;
    }

    tcflush(fd_, TCIOFLUSH);
    LOG_INFO("ODrive: opened " + port + " @ " + std::to_string(baud) + " baud");
    return true;
}

void ODrive::set_velocity_limit(float limit_turns_per_s) {
    vel_limit_ = std::abs(limit_turns_per_s);

    // Set control mode to velocity
    send("w axis0.controller.config.control_mode " +
         std::to_string(CONTROL_MODE_VELOCITY));

    // Apply velocity limit
    send("w axis0.controller.config.vel_limit " +
         std::to_string(vel_limit_));

    LOG_INFO("ODrive: velocity limit set to " + std::to_string(vel_limit_) + " turns/s");
}

void ODrive::enable() {
    send("w axis0.requested_state " +
         std::to_string(AXIS_STATE_CLOSED_LOOP_CONTROL));
    LOG_INFO("ODrive: axis0 → CLOSED_LOOP_CONTROL");
}

void ODrive::set_velocity(float turns_per_s) {
    // Clamp to configured limit
    const float clamped = std::clamp(turns_per_s, -vel_limit_, vel_limit_);
    if (clamped != turns_per_s) {
        LOG_INFO("ODrive: velocity clamped from " + std::to_string(turns_per_s) +
                 " to " + std::to_string(clamped) + " turns/s");
    }
    // ASCII velocity command: "v <axis> <velocity>"
    send("v 0 " + std::to_string(clamped));
}

void ODrive::stop() {
    // Zero velocity first, then go idle
    send("v 0 0");
    send("w axis0.requested_state " + std::to_string(AXIS_STATE_IDLE));
    LOG_INFO("ODrive: axis0 stopped → IDLE");
}

void ODrive::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
        LOG_INFO("ODrive: port closed");
    }
}

// ── Serial helpers ────────────────────────────────────────────────────────────

void ODrive::send(const std::string& cmd) {
    if (fd_ < 0) return;
    const std::string line = cmd + "\n";
    ssize_t written = ::write(fd_, line.c_str(), line.size());
    if (written < 0) {
        LOG_INFO("ODrive: write error — " + std::string(std::strerror(errno)));
    }
}

std::string ODrive::recv_line(int timeout_ms) {
    if (fd_ < 0) return {};

    pollfd pfd{ fd_, POLLIN, 0 };
    std::string result;

    while (true) {
        const int ready = poll(&pfd, 1, timeout_ms);
        if (ready <= 0) break;   // timeout or error

        char c;
        if (::read(fd_, &c, 1) != 1) break;
        if (c == '\n') break;
        if (c != '\r') result += c;
    }
    return result;
}
