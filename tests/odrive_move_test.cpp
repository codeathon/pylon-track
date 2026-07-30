// CLI smoke test for MotionPlanner: move a chain distance at a chosen speed.
// Why: lab bring-up needs distance/speed moves without running a full chase session.
//
// Examples:
//   ./bin/test_odrive_move --config ../config/arena_experiment.json \
//     --distance-mm 500 --speed-mmps 150
//   ./bin/test_odrive_move --config ../config/arena_experiment.json \
//     --distance-mm -300 --duration-ms 2000

#include <cmath>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>

#include "calibrate/setup_util.h"
#include "experiment/arena_config.h"
#include "log/logger.h"
#include "motor/motor_config.h"
#include "motor/motion_planner.h"
#include "motor/prey_motor.h"

namespace {

// Set by SIGINT/SIGTERM so Ctrl+C cancels the trapezoid move cleanly.
MotionPlanner* g_planner = nullptr;
PreyMotor* g_motor = nullptr;

void on_stop_signal(int) {
	if (g_planner) {
		g_planner->cancel();
	}
	if (g_motor) {
		g_motor->stop();
	}
}

void print_usage() {
	std::cerr <<
		"Usage:\n"
		"  test_odrive_move --config <arena_experiment.json>\n"
		"                   --distance-mm <mm>\n"
		"                   (--speed-mmps <mm/s> | --duration-ms <ms>)\n"
		"                   [--accel-mps2 <m/s^2>] [--verbose]\n"
		"\n"
		"Moves the prey chain via MotionPlanner (trapezoid velocity profile).\n"
		"Distance sign selects direction (uses chain_direction_sign from config).\n";
}

struct Args {
	std::string config_path;
	float distance_mm = 0.0f;
	float speed_mmps = 0.0f;
	int duration_ms = 0;
	float accel_mps2 = 0.5f;
	bool verbose = false;
	bool have_distance = false;
	bool have_speed = false;
	bool have_duration = false;
};

bool parse_args(int argc, char** argv, Args& args) {
	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
			args.config_path = argv[++i];
		} else if (std::strcmp(argv[i], "--distance-mm") == 0 && i + 1 < argc) {
			args.distance_mm = std::stof(argv[++i]);
			args.have_distance = true;
		} else if (std::strcmp(argv[i], "--speed-mmps") == 0 && i + 1 < argc) {
			args.speed_mmps = std::stof(argv[++i]);
			args.have_speed = true;
		} else if (std::strcmp(argv[i], "--duration-ms") == 0 && i + 1 < argc) {
			args.duration_ms = std::stoi(argv[++i]);
			args.have_duration = true;
		} else if (std::strcmp(argv[i], "--accel-mps2") == 0 && i + 1 < argc) {
			args.accel_mps2 = std::stof(argv[++i]);
		} else if (std::strcmp(argv[i], "--verbose") == 0) {
			args.verbose = true;
		} else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
			return false;
		} else {
			std::cerr << "Unknown argument: " << argv[i] << '\n';
			return false;
		}
	}
	if (args.config_path.empty() || !args.have_distance) {
		return false;
	}
	if (args.have_speed == args.have_duration) {
		std::cerr << "ERROR: provide exactly one of --speed-mmps or --duration-ms\n";
		return false;
	}
	return true;
}

} // namespace

int main(int argc, char** argv) {
	Args args;
	if (!parse_args(argc, argv, args)) {
		print_usage();
		return 1;
	}

	Logger::instance().set_level(args.verbose ? LogLevel::Debug : LogLevel::Info);

	ArenaExperimentConfig cfg;
	if (!load_arena_experiment_config(args.config_path, cfg)) {
		return 1;
	}

	int duration_ms = args.duration_ms;
	if (args.have_speed) {
		if (std::fabs(args.speed_mmps) < 1e-3f) {
			log_error("test", "--speed-mmps must be non-zero");
			return 1;
		}
		// duration = distance / speed; speed magnitude only (sign is on distance).
		duration_ms = static_cast<int>(
			std::lround(std::fabs(args.distance_mm) / std::fabs(args.speed_mmps) * 1000.0f));
		if (duration_ms < 1) {
			duration_ms = 1;
		}
	}

	// uint8_t node_id must be cast — iostream otherwise prints ASCII (62 → '>').
	std::cout << "Move " << args.distance_mm << " mm in " << duration_ms
		<< " ms (accel " << args.accel_mps2 << " m/s^2)\n"
		<< "CAN " << cfg.motor.can_interface
		<< " node " << static_cast<int>(cfg.motor.node_id)
		<< ", chain_mm_per_turn=" << cfg.motor.chain_mm_per_motor_turn
		<< ", pulley_radius_m=" << cfg.motor.pulley_radius_m
		<< ", direction_sign=" << cfg.motor.chain_direction_sign << '\n';
	if (cfg.motor.chain_mm_per_motor_turn <= 0.0f) {
		log_info("test",
			"chain_mm_per_motor_turn is 0 — using pulley_radius_m fallback; "
			"set motor.chain_mm_per_motor_turn in arena_experiment.json for accurate mm");
	}

	if (!can_interface_up(cfg.motor.can_interface)) {
		log_error("test", can_interface_down_hint(cfg.motor.can_interface));
		return 1;
	}

	PreyMotor motor(prey_motor_from_config(cfg.motor));
	if (!motor.connect()) {
		log_error("test", "PreyMotor connect failed");
		return 1;
	}
	if (!motor.status().heartbeat_ok) {
		log_error("test",
			"No ODrive heartbeat — check ODrive power, can0 UP, and node_id "
			"(candump can0 should show 7C1 heartbeats for node 62)");
		return 1;
	}
	if (!motor.enter_velocity_mode()) {
		log_error("test", "Failed to enter closed-loop velocity mode");
		return 1;
	}

	const float pos_before = motor.read_position_turns();
	MotionPlanner planner;
	// Why: Ctrl+C should cancel the profile and zero velocity, not leave the axis spinning.
	g_planner = &planner;
	g_motor = &motor;
	std::signal(SIGINT, on_stop_signal);
	std::signal(SIGTERM, on_stop_signal);

	std::cout << "Press Ctrl+C to abort the move.\n";
	const bool ok = planner.move_distance_mm_in_time(
		motor, args.distance_mm, duration_ms, args.accel_mps2);
	g_planner = nullptr;
	motor.stop();
	g_motor = nullptr;
	const float pos_after = motor.read_position_turns();
	const float delta_turns = pos_after - pos_before;
	const float delta_mm = motor.turns_to_chain_mm(delta_turns);

	std::cout << "Encoder delta: " << delta_turns << " turns ("
		<< delta_mm << " mm chain)\n";
	if (!ok) {
		log_error("test", "MotionPlanner move failed or cancelled");
		return 1;
	}
	log_info("test", "Move complete");
	return 0;
}
