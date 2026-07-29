#pragma once

#include <fstream>
#include <string>
#include "experiment/trial_state.h"
#include "vision/tracking_frame.h"

// Timestamped CSV logging for experiment sessions (Phase 0).
class SessionRecorder {
public:
	// Creates sessions/<suite>/<timestamp>/ under base_dir.
	void open_session(const std::string& base_dir, const std::string& suite_label);
	bool is_open() const { return telemetry_.is_open(); }

	void log_frame(const TrackingFrame& frame);
	void log_event(const char* name, int64_t host_time_ns, TrialPhase phase);

private:
	std::string session_dir_;
	std::ofstream telemetry_;
	std::ofstream events_;

	void write_telemetry_header();
	void write_events_header();
	static std::string csv_num(double value, int precision = 3);
};
