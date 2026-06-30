#pragma once

#include <string>

// ODrive ASCII-protocol interface over USB-serial (/dev/ttyACM0 or similar).
//
// Supported ODrive firmware: 0.5.x / 0.6.x (ASCII protocol).
// Axis 0 only. Control mode: velocity.
//
// Typical usage:
//   ODrive od;
//   od.open("/dev/ttyACM0");
//   od.set_velocity_limit(10.0f);   // turns/s
//   od.enable();
//   od.set_velocity(5.0f);
//   // ... wait / run tracking loop ...
//   od.stop();
//   od.close();

class ODrive {
public:
    ODrive()  = default;
    ~ODrive() { close(); }

    // Non-copyable
    ODrive(const ODrive&)            = delete;
    ODrive& operator=(const ODrive&) = delete;

    // Open serial port (e.g. "/dev/ttyACM0"). Returns false on error.
    bool open(const std::string& port, int baud = 115200);

    // Configure velocity-control mode and apply the velocity limit [turns/s].
    // Must be called after open() and before enable().
    void set_velocity_limit(float limit_turns_per_s);

    // Put axis into CLOSED_LOOP_CONTROL (axis state 8).
    void enable();

    // Set target velocity [turns/s]. Clamped to ±vel_limit_.
    void set_velocity(float turns_per_s);

    // Set velocity to 0 and put axis back to IDLE (state 1).
    void stop();

    // Close serial port.
    void close();

    bool is_open() const { return fd_ >= 0; }

private:
    void send(const std::string& cmd);
    std::string recv_line(int timeout_ms = 200);

    int   fd_       = -1;
    float vel_limit_ = 10.0f;
};
