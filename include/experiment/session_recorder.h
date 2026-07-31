#pragma once

#include <fstream>
#include <mutex>
#include <string>
#include "experiment/trial_state.h"
#include "vision/tracking_frame.h"

// Timestamped CSV logging for experiment sessions.
class SessionRecorder {
public:
	// Creates sessions/<suite>/<timestamp>/ under base_dir.
	void open_session(const std::string& base_dir, const std::string& suite_label);
	bool is_open() const;

	void log_frame(const TrackingFrame& frame);
	void log_event(const char* name, int64_t host_time_ns, TrialPhase phase);

	// Per-stage latency for hunt analysis (grab/pipeline/decision/motor).
	void log_op_timing(int64_t host_time_ns, const char* op, int64_t duration_us,
		const char* detail = "");

	// Discrete flee arm / complete rows for proximity-driven hunt events.
	void log_chase_event(int64_t host_time_ns, const char* event,
		float distance_mm, float closing_mm_s, float threat, float flee_mm,
		int duration_ms, float peak_turns_s, const char* reason);

	const std::string& session_dir() const { return session_dir_; }

private:
	std::string session_dir_;
	mutable std::mutex mutex_;
	std::ofstream telemetry_;
	std::ofstream events_;
	std::ofstream ops_timing_;
	std::ofstream chase_events_;

	void write_telemetry_header();
	void write_events_header();
	void write_ops_timing_header();
	void write_chase_events_header();
	static std::string csv_num(double value, int precision = 3);
	static std::string csv_escape(const char* text);
};
