#include "experiment/session_recorder.h"
#include "log/logger.h"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <vector>

namespace fs = std::filesystem;

namespace {

std::string timestamp_label() {
	const auto now = std::chrono::system_clock::now();
	const std::time_t tt = std::chrono::system_clock::to_time_t(now);
	std::tm tm_buf{};
	localtime_r(&tt, &tm_buf);
	char buf[32];
	std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H%M%S", &tm_buf);
	return buf;
}

void write_csv_row(std::ofstream& file, const std::vector<std::string>& cells) {
	for (size_t i = 0; i < cells.size(); ++i) {
		if (i > 0) {
			file << ',';
		}
		file << cells[i];
	}
	file << '\n';
	file.flush();
}

} // namespace

void SessionRecorder::open_session(const std::string& base_dir,
	const std::string& suite_label)
{
	const fs::path dir = fs::path(base_dir) / suite_label / timestamp_label();
	std::error_code ec;
	fs::create_directories(dir, ec);
	if (ec) {
		throw std::runtime_error("Cannot create session dir: " + ec.message());
	}
	session_dir_ = dir.string();

	telemetry_.open((dir / "telemetry.csv").string());
	events_.open((dir / "events.csv").string());
	ops_timing_.open((dir / "ops_timing.csv").string());
	chase_events_.open((dir / "chase_events.csv").string());
	if (!telemetry_.is_open() || !events_.is_open()
		|| !ops_timing_.is_open() || !chase_events_.is_open()) {
		throw std::runtime_error("Cannot open session CSV files in " + session_dir_);
	}
	write_telemetry_header();
	write_events_header();
	write_ops_timing_header();
	write_chase_events_header();
	log_info("experiment", "Session recording: " + session_dir_);
}

bool SessionRecorder::is_open() const {
	std::lock_guard<std::mutex> lock(mutex_);
	return telemetry_.is_open();
}

void SessionRecorder::write_telemetry_header() {
	write_csv_row(telemetry_, {
		"frame_index", "camera_ts_ticks", "host_time_ns", "trial_phase",
		"ferret_x_mm", "ferret_y_mm", "ferret_speed_mm_s", "ferret_heading_deg",
		"ferret_valid", "prey_x_mm", "prey_y_mm", "prey_speed_mm_s",
		"prey_heading_deg", "prey_valid", "distance_mm", "bearing_deg",
		"closing_speed_mm_s"
	});
}

void SessionRecorder::write_events_header() {
	write_csv_row(events_, {"host_time_ns", "event", "trial_phase"});
}

void SessionRecorder::write_ops_timing_header() {
	write_csv_row(ops_timing_, {"host_time_ns", "op", "duration_us", "detail"});
}

void SessionRecorder::write_chase_events_header() {
	write_csv_row(chase_events_, {
		"host_time_ns", "event", "distance_mm", "closing_mm_s", "threat",
		"flee_mm", "duration_ms", "peak_turns_s", "reason"
	});
}

std::string SessionRecorder::csv_num(double value, int precision) {
	std::ostringstream oss;
	oss.setf(std::ios::fixed);
	oss.precision(precision);
	oss << value;
	return oss.str();
}

std::string SessionRecorder::csv_escape(const char* text) {
	if (!text) {
		return "";
	}
	std::string s(text);
	// Quote if commas/quotes present so ops detail stays one CSV cell.
	if (s.find_first_of(",\"\n") == std::string::npos) {
		return s;
	}
	std::string out = "\"";
	for (char c : s) {
		if (c == '"') {
			out += "\"\"";
		} else {
			out += c;
		}
	}
	out += '"';
	return out;
}

void SessionRecorder::log_frame(const TrackingFrame& frame) {
	std::lock_guard<std::mutex> lock(mutex_);
	if (!telemetry_.is_open()) {
		return;
	}
	const TrackState& f = frame.ferret.state;
	const TrackState& p = frame.prey.state;
	write_csv_row(telemetry_, {
		std::to_string(frame.frame_index),
		std::to_string(frame.camera_ts_ticks),
		std::to_string(frame.host_time_ns),
		trial_phase_name(frame.trial_phase),
		csv_num(f.pos_mm.x, 1), csv_num(f.pos_mm.y, 1),
		csv_num(f.speed_mm_s, 1), csv_num(f.direction_deg, 1),
		f.valid ? "1" : "0",
		csv_num(p.pos_mm.x, 1), csv_num(p.pos_mm.y, 1),
		csv_num(p.speed_mm_s, 1), csv_num(p.direction_deg, 1),
		p.valid ? "1" : "0",
		csv_num(frame.distance_mm, 1),
		csv_num(frame.bearing_deg, 1),
		csv_num(frame.closing_speed_mm_s, 1)
	});
}

void SessionRecorder::log_event(const char* name, int64_t host_time_ns, TrialPhase phase) {
	std::lock_guard<std::mutex> lock(mutex_);
	if (!events_.is_open()) {
		return;
	}
	write_csv_row(events_, {
		std::to_string(host_time_ns),
		name,
		trial_phase_name(phase)
	});
}

void SessionRecorder::log_op_timing(int64_t host_time_ns, const char* op,
	int64_t duration_us, const char* detail)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!ops_timing_.is_open()) {
		return;
	}
	write_csv_row(ops_timing_, {
		std::to_string(host_time_ns),
		op ? op : "",
		std::to_string(duration_us),
		csv_escape(detail)
	});
}

void SessionRecorder::log_chase_event(int64_t host_time_ns, const char* event,
	float distance_mm, float closing_mm_s, float threat, float flee_mm,
	int duration_ms, float peak_turns_s, const char* reason)
{
	std::lock_guard<std::mutex> lock(mutex_);
	if (!chase_events_.is_open()) {
		return;
	}
	write_csv_row(chase_events_, {
		std::to_string(host_time_ns),
		event ? event : "",
		csv_num(distance_mm, 1),
		csv_num(closing_mm_s, 1),
		csv_num(threat, 3),
		csv_num(flee_mm, 1),
		std::to_string(duration_ms),
		csv_num(peak_turns_s, 3),
		csv_escape(reason)
	});
}
