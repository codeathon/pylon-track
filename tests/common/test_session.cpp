#include "test_session.h"

#include <chrono>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

std::string timestamp_label() {
	const auto now = std::chrono::system_clock::now();
	const std::time_t tt = std::chrono::system_clock::to_time_t(now);
	std::tm tm_buf{};
	localtime_r(&tt, &tm_buf);
	char buf[32];
	// Local time on purpose: labels must match the operator's lab notebook.
	std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H%M%S", &tm_buf);
	return buf;
}

std::string make_session_dir(const std::string& base,
	const std::string& suite,
	const std::string& label)
{
	std::string name = timestamp_label();
	if (!label.empty()) {
		name += "_" + label;
	}
	const fs::path dir = fs::path(base) / suite / name;
	std::error_code ec;
	fs::create_directories(dir, ec);
	if (ec) {
		throw std::runtime_error("Cannot create session dir " + dir.string()
			+ ": " + ec.message());
	}
	return dir.string();
}

CsvWriter::CsvWriter(const std::string& path,
	const std::vector<std::string>& header)
	: file_(path)
{
	if (!file_.is_open()) {
		throw std::runtime_error("Cannot open CSV for writing: " + path);
	}
	write_row(header);
}

void CsvWriter::write_row(const std::vector<std::string>& cells) {
	for (size_t i = 0; i < cells.size(); ++i) {
		if (i > 0) {
			file_ << ',';
		}
		file_ << cells[i];
	}
	file_ << '\n';
	// Flush every row: capture runs are long and interrupted often (Ctrl-C).
	file_.flush();
}

std::string csv_num(double value, int precision) {
	std::ostringstream oss;
	oss.setf(std::ios::fixed);
	oss.precision(precision);
	oss << value;
	return oss.str();
}
