// CLI smoke test for MotionPlanner: move a chain distance at a chosen speed.
// Why: lab bring-up needs distance/speed moves without running a full chase session.
//
// Examples:
//   ./bin/test_odrive_move --config ../config/arena_experiment.json \
//     --distance-mm 500 --speed-mmps 150
//   ./bin/test_odrive_move --config ../config/arena_experiment.json \
//     --vel-turns-s 1 --seconds 5

#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>

#include "calibrate/setup_util.h"
#include "experiment/arena_config.h"
#include "log/logger.h"
#include "motor/motor_config.h"
#include "motor/motion_planner.h"
#include "motor/prey_motor.h"

namespace {

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
		"                   [--accel-mps2 <m/s^2>] [--via-planner]\n"
		"                   [--plan-only] [--verbose]\n"
		"\n"
		"  test_odrive_move --config <arena_experiment.json>\n"
		"                   --vel-turns-s <turns/s> --seconds <s>\n"
		"\n"
		"--plan-only prints the feasibility checklist (no motor motion).\n"
		"Distance mode defaults to the same Set_Input_Vel loop as --vel-turns-s\n"
		"(planned peak held for the full window). --via-planner uses MotionPlanner.\n";
}

struct Args {
	std::string config_path;
	float distance_mm = 0.0f;
	float speed_mmps = 0.0f;
	int duration_ms = 0;
	// Higher default so short/fast lab moves can actually reach cruise.
	float accel_mps2 = 5.0f;
	float vel_turns_s = 0.0f;
	float seconds = 0.0f;
	bool verbose = false;
	bool have_distance = false;
	bool have_speed = false;
	bool have_duration = false;
	bool have_vel_turns = false;
	bool have_seconds = false;
	bool plan_only = false;
	// Why: default distance path must match known-good raw spin; planner is opt-in.
	bool via_planner = false;
};

// Shared spin loop for --vel-turns-s and default distance mode.
// Returns encoder delta (turns). Prints mid-move vel / axis_state / err.
//
// Why: the old loop called blocking status()+heartbeat every tick (~200–400 ms)
// then slept 100 ms — a 0.5 s window only got ~1 Set_Input_Vel before stop(),
// so encoder delta stayed ~0 even though axis_state=8 / err=0. Keep commanding
// at ~50 Hz; sample feedback without blocking the cadence.
float spin_turns_s_for(PreyMotor& motor, float turns_s, float seconds) {
	// Re-assert mode/limits in case GUI left vel_limit=0.
	if (!motor.assert_velocity_control()) {
		log_error("test", "assert_velocity_control (mode+limits) failed");
	}
	const float pos_before = motor.read_position_turns();
	const auto end = std::chrono::steady_clock::now()
		+ std::chrono::milliseconds(static_cast<int>(seconds * 1000.0f));
	float vel_sample = 0.0f;
	float iq_sp = 0.0f;
	float iq_meas = 0.0f;
	uint32_t err = 0;
	uint32_t state = 0;
	uint32_t active_err = 0;
	uint32_t disarm = 0;
	int cmds = 0;
	while (std::chrono::steady_clock::now() < end) {
		if (!motor.set_velocity_turns_s(turns_s)) {
			log_error("test", "Set_Input_Vel failed");
			motor.stop();
			return motor.read_position_turns() - pos_before;
		}
		++cmds;
		float peek = 0.0f;
		if (motor.try_sample_velocity_turns_s(peek, /*timeout_ms=*/0)) {
			vel_sample = peek;
		}
		float iq_a = 0.0f;
		float iq_b = 0.0f;
		if (motor.try_get_iq(iq_a, iq_b, /*timeout_ms=*/0)) {
			iq_sp = iq_a;
			iq_meas = iq_b;
		}
		uint32_t ae = 0;
		uint32_t dr = 0;
		if (motor.try_get_active_errors(ae, dr, /*timeout_ms=*/0)) {
			active_err = ae;
			disarm = dr;
		}
		if (cmds == 1 || cmds % 10 == 0) {
			motor.read_axis_state(err, state);
			std::ostringstream oss;
			oss << "spin cmd#" << cmds
				<< " cmd=" << turns_s
				<< " sample_vel=" << vel_sample
				<< " iq_sp=" << iq_sp
				<< " iq_meas=" << iq_meas
				<< " axis_state=" << state
				<< " hb_err=0x" << std::hex << err
				<< " active_err=0x" << active_err
				<< " disarm=0x" << disarm << std::dec;
			log_info("test", oss.str());
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
	motor.stop();
	vel_sample = motor.status().velocity_turns_s;
	motor.read_axis_state(err, state);
	const float delta = motor.read_position_turns() - pos_before;
	std::cout << "Encoder delta: " << delta << " turns"
		<< " (cmds=" << cmds
		<< ", sample vel " << vel_sample
		<< ", iq_sp=" << iq_sp << ", iq_meas=" << iq_meas
		<< ", axis_state=" << state
		<< ", hb_err=0x" << std::hex << err
		<< ", active_err=0x" << active_err
		<< ", disarm=0x" << disarm << std::dec << ")\n";
	return delta;
}

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
		} else if (std::strcmp(argv[i], "--vel-turns-s") == 0 && i + 1 < argc) {
			args.vel_turns_s = std::stof(argv[++i]);
			args.have_vel_turns = true;
		} else if (std::strcmp(argv[i], "--seconds") == 0 && i + 1 < argc) {
			args.seconds = std::stof(argv[++i]);
			args.have_seconds = true;
		} else if (std::strcmp(argv[i], "--verbose") == 0) {
			args.verbose = true;
		} else if (std::strcmp(argv[i], "--plan-only") == 0) {
			args.plan_only = true;
		} else if (std::strcmp(argv[i], "--via-planner") == 0) {
			args.via_planner = true;
		} else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
			return false;
		} else {
			std::cerr << "Unknown argument: " << argv[i] << '\n';
			return false;
		}
	}
	if (args.config_path.empty()) {
		return false;
	}
	if (args.have_vel_turns || args.have_seconds) {
		return args.have_vel_turns && args.have_seconds
			&& !args.have_distance && !args.have_speed && !args.have_duration;
	}
	if (!args.have_distance) {
		return false;
	}
	if (args.have_speed == args.have_duration) {
		std::cerr << "ERROR: provide exactly one of --speed-mmps or --duration-ms\n";
		return false;
	}
	return true;
}

bool connect_motor(PreyMotor& motor) {
	if (!motor.connect()) {
		log_error("test", "PreyMotor connect failed");
		return false;
	}
	if (!motor.status().heartbeat_ok) {
		log_error("test",
			"No ODrive heartbeat — check power/CAN (candump should show 7C1)");
		return false;
	}
	if (!motor.enter_velocity_mode()) {
		log_error("test", "Failed to enter closed-loop velocity mode");
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

	// Why: lab often edits a different copy than ../config from build/.
	std::error_code ec;
	const auto abs_cfg = std::filesystem::absolute(args.config_path, ec);
	std::cout << "Config file: "
		<< (ec ? args.config_path : abs_cfg.string()) << '\n';
	std::cout << "CAN " << cfg.motor.can_interface
		<< " node " << static_cast<int>(cfg.motor.node_id)
		<< ", chain_mm_per_turn=" << cfg.motor.chain_mm_per_motor_turn
		<< ", pulley_radius_m=" << cfg.motor.pulley_radius_m
		<< ", direction_sign=" << cfg.motor.chain_direction_sign << '\n';

	PreyMotor motor(prey_motor_from_config(cfg.motor));

	// Distance / duration planning (no CAN required for --plan-only).
	if (args.have_distance && !args.have_vel_turns) {
		int duration_ms = args.duration_ms;
		if (args.have_speed) {
			if (std::fabs(args.speed_mmps) < 1e-3f) {
				log_error("test", "--speed-mmps must be non-zero");
				return 1;
			}
			duration_ms = static_cast<int>(std::lround(
				std::fabs(args.distance_mm) / std::fabs(args.speed_mmps) * 1000.0f));
			if (duration_ms < 1) {
				duration_ms = 1;
			}
		}
		// Feasibility checklist (offline) — not what execute sends.
		const ChainMovePlan checklist = MotionPlanner::plan_distance_mm_in_time(
			motor, args.distance_mm, duration_ms, args.accel_mps2);
		std::cout << checklist.summary;
		// Runtime profile (floor + fast ramp) — what move_distance actually runs.
		const ChainMovePlan runtime = MotionPlanner::runtime_plan_distance_mm_in_time(
			motor, args.distance_mm, duration_ms, args.accel_mps2);
		std::cout << "=== Runtime execute profile ===\n" << runtime.summary;
		if (args.plan_only) {
			return runtime.feasible ? 0 : 1;
		}
		if (!runtime.feasible) {
			log_error("test", "Runtime plan not feasible — fix parameters above");
			return 1;
		}
	}

	if (!can_interface_up(cfg.motor.can_interface)) {
		log_error("test", can_interface_down_hint(cfg.motor.can_interface));
		return 1;
	}

	g_motor = &motor;
	std::signal(SIGINT, on_stop_signal);
	std::signal(SIGTERM, on_stop_signal);

	// Raw turns/s path — skips mm scale.
	if (args.have_vel_turns) {
		if (!connect_motor(motor)) {
			return 1;
		}
		std::cout << "Raw spin: " << args.vel_turns_s << " turns/s for "
			<< args.seconds << "s (Ctrl+C to abort)\n";
		const float delta = spin_turns_s_for(motor, args.vel_turns_s, args.seconds);
		g_motor = nullptr;
		if (std::fabs(delta) < 0.1f) {
			log_error("test",
				"Encoder barely moved — this rig needs ~1–1.5 s to spool up "
				"(try --seconds 2). If 2s also fails, check GUI closed-loop "
				"input_vel=1");
			return 1;
		}
		log_info("test", "Raw spin complete");
		return 0;
	}

	int duration_ms = args.duration_ms;
	if (args.have_speed) {
		duration_ms = static_cast<int>(
			std::lround(std::fabs(args.distance_mm) / std::fabs(args.speed_mmps) * 1000.0f));
		if (duration_ms < 1) {
			duration_ms = 1;
		}
	}

	if (!connect_motor(motor)) {
		return 1;
	}

	const ChainMovePlan runtime = MotionPlanner::runtime_plan_distance_mm_in_time(
		motor, args.distance_mm, duration_ms, args.accel_mps2);
	std::cout << "Press Ctrl+C to abort the move.\n";

	float delta_turns = 0.0f;
	float planned_mm = runtime.expected_distance_mm;
	bool ok = true;

	if (args.via_planner) {
		// Opt-in: exercise MotionPlanner::execute_plan (chase path).
		const float pos_before = motor.read_position_turns();
		MotionPlanner planner;
		g_planner = &planner;
		ChainMovePlan executed;
		ok = planner.move_distance_mm_in_time(motor, args.distance_mm,
			duration_ms, args.accel_mps2, /*require_feasible=*/true, &executed);
		g_planner = nullptr;
		motor.stop();
		delta_turns = motor.read_position_turns() - pos_before;
		planned_mm = executed.expected_distance_mm;
		std::cout << "Encoder delta: " << delta_turns << " turns ("
			<< motor.turns_to_chain_mm(delta_turns) << " mm chain); planned "
			<< planned_mm << " mm [--via-planner]\n";
	} else {
		// Default: identical CAN loop to --vel-turns-s (isolates planner bugs).
		std::cout << "Direct spin at planned peak " << runtime.peak_turns_s
			<< " turns/s for " << (duration_ms / 1000.0f) << "s "
			<< "(same loop as --vel-turns-s; use --via-planner for MotionPlanner)\n";
		delta_turns = spin_turns_s_for(motor, runtime.peak_turns_s,
			duration_ms / 1000.0f);
		std::cout << "Chain delta: " << motor.turns_to_chain_mm(delta_turns)
			<< " mm; planned " << planned_mm << " mm\n";
	}
	g_motor = nullptr;

	if (!ok) {
		log_error("test", "MotionPlanner move failed or cancelled");
		return 1;
	}
	// Why: 0.1 turns ≈ 66 mm on this chain — that falsely failed 40–60 mm
	// closed-loop moves that actually tracked. Score in mm vs the request.
	const float delta_mm = motor.turns_to_chain_mm(delta_turns);
	const float req_abs = std::fabs(args.distance_mm);
	const float min_ok_mm = std::max(15.0f, 0.5f * req_abs);
	if (std::fabs(delta_mm) < min_ok_mm) {
		log_error("test",
			"Encoder travel too small (" + std::to_string(delta_mm)
			+ " mm vs request " + std::to_string(args.distance_mm)
			+ " mm). Compare:\n"
			"  ./bin/test_odrive_move --config ... --vel-turns-s 1.8 --seconds 2");
		return 1;
	}
	log_info("test", "Move complete");
	return 0;
}
