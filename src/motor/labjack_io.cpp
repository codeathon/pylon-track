#include "motor/labjack_io.h"

#include "log/logger.h"

#ifdef PYLON_TRACK_HAS_LJM
#include <LabJackM.h>

namespace {

// LJM_ErrorToString writes into a caller buffer (void return in current LJM).
std::string ljm_error_string(int err) {
	char buf[LJM_MAX_NAME_SIZE] = {};
	LJM_ErrorToString(err, buf);
	return buf;
}

} // namespace
#endif

LabJackIo::LabJackIo(const LabJackConfig& cfg) : cfg_(cfg) {}

double LabJackIo::logical_to_line(double logical) const {
	const bool on = logical >= 0.5;
	if (cfg_.active_high) {
		return on ? 1.0 : 0.0;
	}
	return on ? 0.0 : 1.0;
}

bool LabJackIo::open() {
	if (handle_ >= 0) {
		return true;
	}
#ifdef PYLON_TRACK_HAS_LJM
	const int err = LJM_OpenS(cfg_.device_type.c_str(),
		cfg_.connection.c_str(), cfg_.identifier.c_str(), &handle_);
	if (err != LJME_NOERROR) {
		log_error("motor", "LabJack open failed: " + ljm_error_string(err));
		handle_ = -1;
		return false;
	}
	log_info("motor", "LabJack connected (" + cfg_.device_type + " " + cfg_.connection + ")");
	return true;
#else
	log_error("motor",
		"LabJack LJM not linked — rebuild with -DENABLE_LABJACK=ON and LJM installed");
	return false;
#endif
}

void LabJackIo::close() {
#ifdef PYLON_TRACK_HAS_LJM
	if (handle_ >= 0) {
		LJM_Close(handle_);
	}
#endif
	handle_ = -1;
}

bool LabJackIo::write_dio(double value) {
	if (handle_ < 0) {
		return false;
	}
#ifdef PYLON_TRACK_HAS_LJM
	const double line = logical_to_line(value);
	const int err = LJM_eWriteName(handle_, cfg_.dio_pin.c_str(), line);
	if (err != LJME_NOERROR) {
		log_error("motor", "LabJack write " + cfg_.dio_pin + " failed: "
			+ ljm_error_string(err));
		return false;
	}
	return true;
#else
	(void)value;
	return false;
#endif
}

bool LabJackIo::read_dio(double& value) const {
	if (handle_ < 0) {
		return false;
	}
#ifdef PYLON_TRACK_HAS_LJM
	double raw = 0.0;
	const int err = LJM_eReadName(handle_, cfg_.dio_pin.c_str(), &raw);
	if (err != LJME_NOERROR) {
		return false;
	}
	if (cfg_.active_high) {
		value = raw;
	} else {
		value = (raw < 0.5) ? 1.0 : 0.0;
	}
	return true;
#else
	(void)value;
	return false;
#endif
}
