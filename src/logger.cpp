#include "logger.h"

#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <memory>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <sstream>

namespace {

std::mutex g_log_mutex;
std::unique_ptr<std::ofstream> g_log_file;

const char* level_name(LogLevel level) {
	switch (level) {
	case LogLevel::Debug: return "DEBUG";
	case LogLevel::Info:  return "INFO";
	case LogLevel::Warn:  return "WARN";
	case LogLevel::Error: return "ERROR";
	}
	return "UNKNOWN";
}

std::string format_timestamp() {
	const auto now = std::chrono::system_clock::now();
	const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		now.time_since_epoch()) % 1000;
	const std::time_t tt = std::chrono::system_clock::to_time_t(now);
	std::tm tm_buf{};
	localtime_r(&tt, &tm_buf);

	std::ostringstream oss;
	oss << std::put_time(&tm_buf, "%Y-%m-%d %H:%M:%S")
		<< '.' << std::setfill('0') << std::setw(3) << ms.count();
	return oss.str();
}

std::string format_line(LogLevel level, const char* component, const std::string& message) {
	std::ostringstream oss;
	oss << '[' << format_timestamp() << "] ["
		<< level_name(level) << "] [" << component << "] " << message;
	return oss.str();
}

} // namespace

Logger& Logger::instance() {
	static Logger logger;
	return logger;
}

void Logger::set_level(LogLevel min_level) {
	std::lock_guard<std::mutex> lock(g_log_mutex);
	min_level_ = min_level;
}

void Logger::set_log_file(const std::string& path) {
	std::lock_guard<std::mutex> lock(g_log_mutex);
	g_log_file.reset();

	if (path.empty()) {
		return;
	}

	const std::filesystem::path file_path(path);
	if (file_path.has_parent_path()) {
		std::error_code ec;
		std::filesystem::create_directories(file_path.parent_path(), ec);
	}

	g_log_file = std::make_unique<std::ofstream>(path, std::ios::app);
	if (!g_log_file->is_open()) {
		std::cerr << "Failed to open log file: " << path << '\n';
		g_log_file.reset();
	}
}

void Logger::log(LogLevel level, const char* component, const std::string& message) {
	std::lock_guard<std::mutex> lock(g_log_mutex);
	if (static_cast<int>(level) < static_cast<int>(min_level_)) {
		return;
	}
	write_line(level, component, message);
}

void Logger::write_line(LogLevel level, const char* component, const std::string& message) {
	const std::string line = format_line(level, component, message);

	std::ostream& console = (level == LogLevel::Error || level == LogLevel::Warn)
		? std::cerr : std::cout;
	console << line << '\n';
	console.flush();

	if (g_log_file && g_log_file->is_open()) {
		*g_log_file << line << '\n';
		g_log_file->flush();
	}
}

void log_debug(const char* component, const std::string& message) {
	Logger::instance().log(LogLevel::Debug, component, message);
}

void log_info(const char* component, const std::string& message) {
	Logger::instance().log(LogLevel::Info, component, message);
}

void log_warn(const char* component, const std::string& message) {
	Logger::instance().log(LogLevel::Warn, component, message);
}

void log_error(const char* component, const std::string& message) {
	Logger::instance().log(LogLevel::Error, component, message);
}
