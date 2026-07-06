#pragma once

#include <string>

// Low-level LabJack LJM digital I/O wrapper (trap door relay/solenoid).
// When LJM is not installed at build time, calls log and return false.
struct LabJackConfig {
	std::string device_type = "T7";
	std::string connection = "ANY";
	std::string identifier = "ANY";
	// LJM register name, e.g. "FIO0", "EIO0", "CIO0".
	std::string dio_pin = "FIO0";
	bool active_high = true;
};

class LabJackIo {
public:
	explicit LabJackIo(const LabJackConfig& cfg);

	bool open();
	void close();
	bool is_open() const { return handle_ >= 0; }

	// Write digital line state: value 1.0 = ON, 0.0 = OFF (after active_high mapping).
	bool write_dio(double value);
	bool read_dio(double& value) const;

private:
	LabJackConfig cfg_;
	int handle_ = -1;

	double logical_to_line(double logical) const;
};
