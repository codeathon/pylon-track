#include "experiment/session_recorder.h"
#include "log/logger.h"

#include <chrono>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

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
	if (!telemetry_.is_open() || !events_.is_open()) {
		throw std::runtime_error("Cannot open session CSV files in " + session_dir_);
	}
	write_telemetry_header();
	write_events_header();
	log_info("experiment", "Session recording: " + session_dir_);
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

std::string SessionRecorder::csv_num(double value, int precision) {
	std::ostringstream oss;
	oss.setf(std::ios::fixed);
	oss.precision(precision);
	oss << value;
	return oss.str();
}

void SessionRecorder::log_frame(const TrackingFrame& frame) {
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
	if (!events_.is_open()) {
		return;
	}
	write_csv_row(events_, {
		std::to_string(host_time_ns),
		name,
		trial_phase_name(phase)
	});
}
