#include "experiment/op_timing.h"

#include "experiment/session_recorder.h"

OpTimer::OpTimer(SessionRecorder* recorder, const char* op, const char* detail)
	: recorder_(recorder)
	, op_(op ? op : "")
	, detail_(detail ? detail : "")
	, t0_(std::chrono::steady_clock::now())
	, host_time_ns_(host_now_ns())
{
}

OpTimer::~OpTimer() {
	if (!recorder_ || !recorder_->is_open()) {
		return;
	}
	const int64_t duration_us = std::chrono::duration_cast<std::chrono::microseconds>(
		std::chrono::steady_clock::now() - t0_).count();
	recorder_->log_op_timing(host_time_ns_, op_, duration_us, detail_.c_str());
}

void record_op_duration(SessionRecorder* recorder, const char* op,
	int64_t host_time_ns, int64_t duration_us, const char* detail)
{
	// Why: this is a free function — use the parameter, not OpTimer::recorder_.
	if (!recorder || !recorder->is_open()) {
		return;
	}
	recorder->log_op_timing(host_time_ns, op, duration_us, detail ? detail : "");
}
