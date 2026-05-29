#pragma once

#include <string>

enum class LogLevel {
	Debug,
	Info,
	Warn,
	Error
};

class Logger {
public:
	static Logger& instance();

	void set_level(LogLevel min_level);
	void set_log_file(const std::string& path);
	void log(LogLevel level, const char* component, const std::string& message);

private:
	Logger() = default;

	LogLevel min_level_ = LogLevel::Info;

	void write_line(LogLevel level, const char* component, const std::string& message);
};

void log_debug(const char* component, const std::string& message);
void log_info(const char* component, const std::string& message);
void log_warn(const char* component, const std::string& message);
void log_error(const char* component, const std::string& message);
