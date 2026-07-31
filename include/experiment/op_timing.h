#pragma once

#include <chrono>
#include <cstdint>
#include <string>

class SessionRecorder;

// Wall-clock host time for correlating ops with telemetry.csv rows.
inline int64_t host_now_ns() {
	return std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::system_clock::now().time_since_epoch()).count();
}

// RAII timer: on destruction logs duration_us to SessionRecorder (if non-null).
// Why: hunt analysis needs per-stage latency without scattering clock math.
class OpTimer {
public:
	OpTimer(SessionRecorder* recorder, const char* op, const char* detail = "");
	~OpTimer();

	OpTimer(const OpTimer&) = delete;
	OpTimer& operator=(const OpTimer&) = delete;

	// Optional mid-scope detail update before dtor logs.
	void set_detail(const char* detail) { detail_ = detail ? detail : ""; }

private:
	SessionRecorder* recorder_ = nullptr;
	const char* op_ = "";
	std::string detail_;
	std::chrono::steady_clock::time_point t0_{};
	int64_t host_time_ns_ = 0;
};

// Non-RAII helper when start/end are in different scopes.
void record_op_duration(SessionRecorder* recorder, const char* op,
	int64_t host_time_ns, int64_t duration_us, const char* detail = "");
