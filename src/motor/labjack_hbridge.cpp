#include "motor/labjack_hbridge.h"

#include "log/logger.h"

#ifdef PYLON_TRACK_HAS_LJM
#include <LabJackM.h>

namespace {

// LJM_ErrorToString writes into a caller buffer (void return in current LJM) —
// treating it as if it returned a string was a bug found via a real build.
std::string ljm_error_string(int err) {
	char buf[LJM_MAX_NAME_SIZE] = {};
	LJM_ErrorToString(err, buf);
	return buf;
}

} // namespace
#endif

LabjackHBridge::LabjackHBridge(const LabjackHBridgeConfig& cfg) : cfg_(cfg) {}

bool LabjackHBridge::open() {
	if (handle_ >= 0) {
		return true;
	}
#ifdef PYLON_TRACK_HAS_LJM
	const int err = LJM_OpenS(cfg_.device_type.c_str(),
		cfg_.connection.c_str(), cfg_.identifier.c_str(), &handle_);
	if (err != LJME_NOERROR) {
		log_error("motor", "LabjackHBridge open failed: " + ljm_error_string(err));
		handle_ = -1;
		return false;
	}
	drive(0); // start stopped — never leave the bridge in an undefined state
	log_info("motor", "LabjackHBridge connected (" + cfg_.device_type + " " + cfg_.connection + ")");
	return true;
#else
	log_error("motor",
		"LabJack LJM not linked — rebuild with -DENABLE_LABJACK=ON and LJM installed");
	return false;
#endif
}

void LabjackHBridge::close() {
#ifdef PYLON_TRACK_HAS_LJM
	if (handle_ >= 0) {
		drive(0);
		LJM_Close(handle_);
	}
#endif
	handle_ = -1;
}

bool LabjackHBridge::drive(int dir) {
	if (handle_ < 0) {
		return false;
	}
#ifdef PYLON_TRACK_HAS_LJM
	// De-energize both lines first — if the two writes below landed in the
	// opposite order there would be a moment with both pins high (a short).
	int err = LJM_eWriteName(handle_, cfg_.pin_a.c_str(), 0.0);
	err |= LJM_eWriteName(handle_, cfg_.pin_b.c_str(), 0.0);
	if (dir > 0) {
		err |= LJM_eWriteName(handle_, cfg_.pin_a.c_str(), cfg_.high_voltage);
	} else if (dir < 0) {
		err |= LJM_eWriteName(handle_, cfg_.pin_b.c_str(), cfg_.high_voltage);
	}
	if (err != LJME_NOERROR) {
		log_error("motor", "LabjackHBridge write failed");
		return false;
	}
	return true;
#else
	(void)dir;
	return false;
#endif
}
