#include <atomic>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>

#include "calibrate/setup_options.h"
#include "calibrate/setup_runner.h"
#include "experiment/state_manager.h"
#include "log/logger.h"

static std::atomic<bool> g_running{true};

static void signal_handler(int) {
	g_running.store(false);
}

enum class Command {
	Setup,
	Run,
	Unknown
};

struct CliOptions {
	Command command = Command::Unknown;
	bool verbose = false;
	bool enable_display = false;
	bool disable_calib = false;
	bool skip_interactive = false;
	std::string log_file;
	std::string arena_config;
	std::string session_dir = "sessions";
	std::string camera_config;
	std::string calib_path;
	std::string only_step;
};

static void print_usage() {
	std::cerr <<
		"Usage:\n"
		"  arena_experiment setup --config <arena_experiment.json> [--display]\n"
		"  arena_experiment run --config <arena_experiment.json> [--session <dir>]\n"
		"\n"
		"  setup  — one-time rig setup (arena masks, camera, ODrive, LabJack)\n"
		"  run    — new experiment session (requires setup first)\n"
		"\n"
		"Options:\n"
		"  --only <arena|camera|odrive|labjack>  run one setup step only\n"
		"  --skip-interactive                      non-interactive setup (CI)\n"
		"  --display                               required for arena/camera setup\n"
		"  --no-calib                              run without calib.npz (dev)\n"
		"  --camera-config <path>  --calib <path>  --verbose  --log-file <path>\n";
}

static bool parse_args(int argc, char** argv, CliOptions& opts) {
	if (argc < 2) {
		return false;
	}
	const std::string cmd = argv[1];
	if (cmd == "setup") {
		opts.command = Command::Setup;
	} else if (cmd == "run") {
		opts.command = Command::Run;
	} else {
		return false;
	}

	for (int i = 2; i < argc; ++i) {
		if (std::strcmp(argv[i], "--config") == 0) {
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
		} else if (std::strcmp(argv[i], "--skip-interactive") == 0) {
			opts.skip_interactive = true;
		} else if (std::strcmp(argv[i], "--only") == 0) {
			if (i + 1 >= argc) return false;
			opts.only_step = argv[++i];
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
		print_usage();
		return 1;
	}

	Logger& logger = Logger::instance();
	logger.set_level(cli.verbose ? LogLevel::Debug : LogLevel::Info);
	if (!cli.log_file.empty()) {
		logger.set_log_file(cli.log_file);
	}

	// Installed before either path runs — setup's live hardware tests
	// (ODrive test spin, LabJack shuttle wobble) also need to react to
	// Ctrl-C and stop the motor, not just the run/chase path.
	std::signal(SIGINT, signal_handler);
	std::signal(SIGTERM, signal_handler);

	if (cli.command == Command::Setup) {
		if (cli.enable_display && std::getenv("DISPLAY") == nullptr) {
			log_error("main", "--display requires DISPLAY");
			return 1;
		}
		SetupOptions opts;
		opts.argv0 = argv[0];
		opts.arena_config_path = cli.arena_config;
		opts.camera_config_path = cli.camera_config;
		opts.calib_path = cli.calib_path;
		opts.verbose = cli.verbose;
		opts.skip_interactive = cli.skip_interactive;
		opts.display = cli.enable_display;
		opts.running = &g_running;
		if (!cli.only_step.empty()) {
			opts.only = parse_setup_step(cli.only_step);
		}
		SetupRunner runner(std::move(opts));
		return runner.run();
	}

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
	opts.log_file = cli.log_file;

	ExperimentStateManager manager(opts);
	return manager.run(g_running);
}
