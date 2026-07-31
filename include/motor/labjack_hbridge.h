#pragma once

#include <string>

// Two-pin H-bridge control over a LabJack analog output (FIO4/FIO5 by
// default — these are analog-capable pins, not fixed digital DIO, so
// "low"/"high" are real voltage levels, not a boolean write).
// dir > 0: pin_a = high_voltage, pin_b = 0V  (spins one direction).
// dir < 0: pin_a = 0V, pin_b = high_voltage  (spins the other direction).
// dir == 0: both 0V (stopped). Both pins at high_voltage shorts the bridge —
// drive() always de-energizes both lines before energizing the new
// direction so the two writes can never leave both pins high at once.
struct LabjackHBridgeConfig {
	std::string device_type = "T7";
	std::string connection = "ANY";
	std::string identifier = "ANY";
	std::string pin_a = "FIO4";
	std::string pin_b = "FIO5";
	double high_voltage = 5.0;
};

class LabjackHBridge {
public:
	explicit LabjackHBridge(const LabjackHBridgeConfig& cfg);

	bool open();
	void close();
	bool is_open() const { return handle_ >= 0; }

	// dir: +1, -1, or 0 (stop). Any other value is treated as 0.
	bool drive(int dir);

private:
	LabjackHBridgeConfig cfg_;
	int handle_ = -1;
};
