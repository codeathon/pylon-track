#include <atomic>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>

#include "experiment/state_manager.h"
#include "log/logger.h"

static std::atomic<bool> g_running{true};

static void signal_handler(int) {
	g_running.store(false);
}

struct CliOptions {
	bool verbose = false;
	bool enable_display = false;
	bool disable_calib = false;
	bool skip_motor_test = false;
	std::string log_file;
	std::string arena_config;
	std::string session_dir = "sessions";
	std::string camera_config;
	std::string calib_path;
};

static bool parse_args(int argc, char** argv, CliOptions& opts) {
	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "run") == 0) {
			continue;
		} else if (std::strcmp(argv[i], "--config") == 0) {
			if (i + 1 >= argc) return false;
			opts.arena_config = argv[++i];
		} else if (std::strcmp(argv[i], "--session") == 0) {
			if (i + 1 >= argc) return false;
			opts.session_dir = argv[++i];
		} else if (std::strcmp(argv[i], "--display") == 0) {
			opts.enable_display = true;
		} else if (std::strcmp(argv[i], "--verbose") == 0) {
			opts.verbose = true;
		} else if (std::strcmp(argv[i], "--log-file") == 0) {
			if (i + 1 >= argc) return false;
			opts.log_file = argv[++i];
		} else if (std::strcmp(argv[i], "--camera-config") == 0) {
			if (i + 1 >= argc) return false;
			opts.camera_config = argv[++i];
		} else if (std::strcmp(argv[i], "--calib") == 0) {
			if (i + 1 >= argc) return false;
			opts.calib_path = argv[++i];
		} else if (std::strcmp(argv[i], "--no-calib") == 0) {
			opts.disable_calib = true;
		} else if (std::strcmp(argv[i], "--skip-motor-test") == 0) {
			opts.skip_motor_test = true;
		} else {
			std::cerr << "ERROR: Unknown argument: " << argv[i] << '\n';
			return false;
		}
	}
	return true;
}

int main(int argc, char** argv) {
	CliOptions cli;
	if (!parse_args(argc, argv, cli)) {
		std::cerr << "Usage: arena_experiment [run] --config <arena_experiment.json>\n"
			"  [--session <dir>] [--display] [--skip-motor-test]\n";
		return 1;
	}

	Logger& logger = Logger::instance();
	logger.set_level(cli.verbose ? LogLevel::Debug : LogLevel::Info);
	if (!cli.log_file.empty()) {
		logger.set_log_file(cli.log_file);
	}

	std::signal(SIGINT, signal_handler);
	std::signal(SIGTERM, signal_handler);

	if (cli.enable_display && std::getenv("DISPLAY") == nullptr) {
		log_error("main", "--display requires DISPLAY");
		return 1;
	}

	ExperimentOptions opts;
	opts.argv0 = argv[0];
	opts.arena_config_path = cli.arena_config;
	opts.session_dir = cli.session_dir;
	opts.camera_config_path = cli.camera_config;
	opts.calib_path = cli.calib_path;
	opts.disable_calib = cli.disable_calib;
	opts.enable_display = cli.enable_display;
	opts.verbose = cli.verbose;
	opts.skip_motor_test = cli.skip_motor_test;
	opts.log_file = cli.log_file;

	ExperimentStateManager manager(opts);
	return manager.run(g_running);
}
