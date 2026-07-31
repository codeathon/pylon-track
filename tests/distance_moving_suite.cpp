// Motor-only distance/duration move test — no camera involved.
//
// Moves the prey chain motor a signed chain distance (mm) over a specified
// duration (s), using MotionPlanner's trapezoidal velocity profile, then
// reports the actual measured distance traveled against what was requested.
// Linux only (SocketCAN), matches the rest of this project.
//
// Usage:
//   test_distance_moving --distance-mm <mm> --duration-s <s>
//       [--config <arena_experiment.json>] [--accel <mps2>] [--verbose]
//
// Example: move 500mm forward over 2 seconds
//   test_distance_moving --distance-mm 500 --duration-s 2

#include <atomic>
#include <csignal>
#include <cstring>
#include <iostream>
#include <optional>
#include <string>

#include "calibrate/setup_util.h"
#include "experiment/arena_config.h"
#include "log/logger.h"
#include "motor/motion_planner.h"
#include "motor/motor_config.h"
#include "motor/prey_motor.h"

namespace {

MotionPlanner* g_planner = nullptr;

void signal_handler(int) {
	if (g_planner) {
		g_planner->cancel();
	}
}

struct Args {
	std::optional<float> distance_mm;
	std::optional<float> duration_s;
	float accel_mps2 = 0.5f;
	std::string config_path;
	bool verbose = false;
};

void print_usage() {
	std::cerr <<
		"Usage: test_distance_moving --distance-mm <mm> --duration-s <s>\n"
		"           [--config <arena_experiment.json>] [--accel <mps2>] [--verbose]\n"
		"\n"
		"  Moves the prey chain motor the given signed distance (mm) over the\n"
		"  given duration (s) using a trapezoidal velocity profile, then reports\n"
		"  the actual measured distance traveled. Ctrl-C aborts and stops the motor.\n";
}

bool parse_args(int argc, char** argv, Args& args) {
	try {
		for (int i = 1; i < argc; ++i) {
			if (std::strcmp(argv[i], "--distance-mm") == 0 && i + 1 < argc) {
				args.distance_mm = std::stof(argv[++i]);
			} else if (std::strcmp(argv[i], "--duration-s") == 0 && i + 1 < argc) {
				args.duration_s = std::stof(argv[++i]);
			} else if (std::strcmp(argv[i], "--accel") == 0 && i + 1 < argc) {
				args.accel_mps2 = std::stof(argv[++i]);
			} else if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
				args.config_path = argv[++i];
			} else if (std::strcmp(argv[i], "--verbose") == 0) {
				args.verbose = true;
			} else {
				std::cerr << "Unknown argument: " << argv[i] << '\n';
				return false;
			}
		}
	} catch (const std::exception& e) {
		std::cerr << "Invalid numeric argument: " << e.what() << '\n';
		return false;
	}
	return true;
}

} // namespace

int main(int argc, char** argv) {
	Args args;
	if (!parse_args(argc, argv, args) || !args.distance_mm || !args.duration_s) {
		print_usage();
		return 1;
	}
	if (*args.duration_s <= 0.0f) {
		std::cerr << "--duration-s must be > 0\n";
		return 1;
	}

	Logger::instance().set_level(args.verbose ? LogLevel::Debug : LogLevel::Info);

	const std::string config_path = resolve_arena_config_path(argv[0], args.config_path);
	if (config_path.empty()) {
		log_error("distance_test", "No arena config found — pass --config <path>");
		return 1;
	}
	ArenaExperimentConfig cfg;
	if (!load_arena_experiment_config(config_path, cfg)) {
		return 1;
	}
	if (cfg.motor.chain_mm_per_motor_turn <= 0.0f) {
		log_error("distance_test",
			"motor.chain_mm_per_motor_turn missing — run: arena_experiment setup --only odrive");
		return 1;
	}

	if (!can_interface_up(cfg.motor.can_interface)) {
		log_error("distance_test", "CAN interface " + cfg.motor.can_interface
			+ " is not up — run: sudo ip link set " + cfg.motor.can_interface
			+ " up type can bitrate 250000");
		return 1;
	}

	PreyMotor motor(prey_motor_from_config(cfg.motor));
	if (!motor.connect()) {
		log_error("distance_test", "Prey motor connect failed");
		return 1;
	}
	if (!motor.status().heartbeat_ok) {
		log_error("distance_test", "Prey motor heartbeat failed");
		return 1;
	}
	if (!motor.enter_velocity_mode()) {
		log_error("distance_test", "Failed to enter closed-loop velocity mode over CAN");
		return 1;
	}

	MotionPlanner planner;
	g_planner = &planner;
	std::signal(SIGINT, signal_handler);
	std::signal(SIGTERM, signal_handler);

	const float pos_before_turns = motor.read_position_turns();
	const int duration_ms = static_cast<int>(*args.duration_s * 1000.0f);

	std::cout << "Moving " << *args.distance_mm << " mm over " << *args.duration_s
		<< " s (max accel " << args.accel_mps2 << " m/s^2)... Ctrl-C to abort.\n";

	const bool completed = planner.move_distance_mm_in_time(
		motor, *args.distance_mm, duration_ms, args.accel_mps2);

	const float pos_after_turns = motor.read_position_turns();
	const float actual_mm = motor.turns_to_chain_mm(pos_after_turns - pos_before_turns);

	std::cout << (completed ? "Move complete" : "Move cancelled") << '\n'
		<< "Requested: " << *args.distance_mm << " mm\n"
		<< "Actual:    " << actual_mm << " mm\n"
		<< "Error:     " << (actual_mm - *args.distance_mm) << " mm\n";

	return completed ? 0 : 1;
}
