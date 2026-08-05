// Hardware-in-the-loop chase hunt smoke: camera + dual track + MotionPlanner flee.
//
// Why: test_hunt_sim is motor-only; test_latency/telemetry are camera-only.
// This joins both so a lab run can prove: camera on → track ferret+prey →
// planner flees a chosen distance/time → encoder travel is within tolerance.
//
// Usage:
//   ./bin/test_chase_hunt --config ../config/arena_experiment.json \
//     [--distance-mm 200] [--duration-ms 2000] [--max-accel 50] \
//     [--warmup-s 10] [--track-timeout-s 60] [--dry-run] [--verbose]
//
// Protocol: empty arena during warmup, then place ferret + prey (or stand-ins)
// so both tracks go valid. Ctrl-C aborts cleanly.

#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>
#include <thread>

#include <pylon/BaslerUniversalInstantCamera.h>
#include <pylon/PylonIncludes.h>

#include "calibrate/setup_util.h"
#include "camera/camera_calib.h"
#include "camera/camera_config.h"
#include "camera/camera_settings.h"
#include "experiment/arena_config.h"
#include "log/logger.h"
#include "motor/lab_motion_limits.h"
#include "motor/motion_planner.h"
#include "motor/motor_config.h"
#include "motor/prey_motor.h"
#include "tracker/ferret_tracker.h"
#include "vision/camera_tracking_service.h"
#include "vision/tracking_frame.h"

namespace {

MotionPlanner* g_planner = nullptr;
std::atomic<bool> g_stop{false};

void signal_handler(int) {
	g_stop.store(true);
	if (g_planner) {
		g_planner->cancel();
	}
}

struct Args {
	std::string arena_config;
	std::string camera_config;
	std::string calib_path;
	float distance_mm = LabMotionLimits::kChaseMinFleeMm;
	int duration_ms = 2000;
	float max_accel_mps2 = LabMotionLimits::kFloorRampAccelMps2;
	double warmup_s = 10.0;
	double track_timeout_s = 60.0;
	int stable_frames = 30; // match experiment identity gate
	float gsd_mm_px = GSD_MM_PX;
	bool dry_run = false;
	bool verbose = false;
	bool disable_calib = false;
};

void print_usage() {
	std::cerr <<
		"Usage: test_chase_hunt --config <arena_experiment.json>\n"
		"  [--camera-config <path>] [--calib <calib.npz>] [--no-calib]\n"
		"  [--distance-mm 200] [--duration-ms 2000] [--max-accel 50]\n"
		"  [--warmup-s 10] [--track-timeout-s 60] [--stable-frames 30]\n"
		"  [--gsd 1.035] [--dry-run] [--verbose]\n"
		"\n"
		"  1) Open Basler camera and start grabbing\n"
		"  2) Connect ODrive (CAN up + heartbeat + velocity mode)\n"
		"  3) Wait until ferret+prey are both valid (stable-frames)\n"
		"  4) MotionPlanner flees --distance-mm in --duration-ms\n"
		"  5) Score encoder travel vs request (lab tolerance)\n";
}

bool parse_args(int argc, char** argv, Args& args) {
	try {
		for (int i = 1; i < argc; ++i) {
			if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
				args.arena_config = argv[++i];
			} else if (std::strcmp(argv[i], "--camera-config") == 0 && i + 1 < argc) {
				args.camera_config = argv[++i];
			} else if (std::strcmp(argv[i], "--calib") == 0 && i + 1 < argc) {
				args.calib_path = argv[++i];
			} else if (std::strcmp(argv[i], "--no-calib") == 0) {
				args.disable_calib = true;
			} else if (std::strcmp(argv[i], "--distance-mm") == 0 && i + 1 < argc) {
				args.distance_mm = std::stof(argv[++i]);
			} else if (std::strcmp(argv[i], "--duration-ms") == 0 && i + 1 < argc) {
				args.duration_ms = std::stoi(argv[++i]);
			} else if (std::strcmp(argv[i], "--max-accel") == 0 && i + 1 < argc) {
				args.max_accel_mps2 = std::stof(argv[++i]);
			} else if (std::strcmp(argv[i], "--warmup-s") == 0 && i + 1 < argc) {
				args.warmup_s = std::stod(argv[++i]);
			} else if (std::strcmp(argv[i], "--track-timeout-s") == 0 && i + 1 < argc) {
				args.track_timeout_s = std::stod(argv[++i]);
			} else if (std::strcmp(argv[i], "--stable-frames") == 0 && i + 1 < argc) {
				args.stable_frames = std::stoi(argv[++i]);
			} else if (std::strcmp(argv[i], "--gsd") == 0 && i + 1 < argc) {
				args.gsd_mm_px = std::stof(argv[++i]);
			} else if (std::strcmp(argv[i], "--dry-run") == 0) {
				args.dry_run = true;
			} else if (std::strcmp(argv[i], "--verbose") == 0) {
				args.verbose = true;
			} else if (std::strcmp(argv[i], "--help") == 0) {
				return false;
			} else {
				std::cerr << "Unknown argument: " << argv[i] << '\n';
				return false;
			}
		}
	} catch (const std::exception& e) {
		std::cerr << "Invalid numeric argument: " << e.what() << '\n';
		return false;
	}
	if (args.duration_ms <= 0 || args.max_accel_mps2 <= 0.0f
		|| args.stable_frames < 1 || args.warmup_s < 0.0
		|| args.track_timeout_s <= 0.0) {
		std::cerr << "Invalid numeric ranges\n";
		return false;
	}
	// Why: sub-~80 mm bursts are flaky on this rig (coast lead ≈ request).
	if (std::fabs(args.distance_mm) < LabMotionLimits::kShortestReliableBurstMm) {
		std::cerr << "distance-mm |" << args.distance_mm
			<< "| below shortest reliable burst ("
			<< LabMotionLimits::kShortestReliableBurstMm << " mm)\n";
		return false;
	}
	return true;
}

// --- Phase 1: camera open + grabbing ---------------------------------------

bool open_camera(Pylon::CBaslerUniversalInstantCamera& camera,
	const Args& args, char** argv, CameraSettings& settings)
{
	const std::string config_path =
		resolve_camera_config_path(argv[0], args.camera_config);
	log_info("chase_hunt", "Loading camera config: " + config_path);
	if (!load_camera_config(config_path, settings)) {
		return false;
	}
	try {
		configure_camera(camera, settings);
	} catch (const std::exception& e) {
		log_error("chase_hunt",
			std::string("Camera configuration failed: ") + e.what());
		return false;
	}
	if (!camera.IsOpen()) {
		log_error("chase_hunt", "Camera not open after configure");
		return false;
	}
	log_info("chase_hunt", "Camera OK: "
		+ std::string(camera.GetDeviceInfo().GetModelName())
		+ " (grabbing next)");
	return true;
}

// --- Phase 2: motor connect + velocity mode --------------------------------

bool connect_motor(const ArenaExperimentConfig& cfg, PreyMotor& motor) {
	if (!can_interface_up(cfg.motor.can_interface)) {
		log_error("chase_hunt", "CAN interface " + cfg.motor.can_interface
			+ " is not up — " + can_interface_down_hint(cfg.motor.can_interface));
		return false;
	}
	if (!motor.connect()) {
		log_error("chase_hunt", "Prey motor connect failed");
		return false;
	}
	if (!motor.status().heartbeat_ok) {
		log_error("chase_hunt", "Prey motor heartbeat failed");
		return false;
	}
	if (!motor.enter_velocity_mode()) {
		log_error("chase_hunt", "Failed to enter closed-loop velocity mode");
		return false;
	}
	log_info("chase_hunt", "Motor OK: can=" + cfg.motor.can_interface
		+ " node=" + std::to_string(cfg.motor.node_id)
		+ " mm/turn=" + std::to_string(cfg.motor.chain_mm_per_motor_turn));
	return true;
}

// --- Phase 3: wait for dual animal lock ------------------------------------

bool wait_for_dual_track(CameraTrackingService& tracker,
	const Args& args, TrackingFrame& out_frame)
{
	log_info("chase_hunt",
		"Waiting for ferret+prey both valid ("
		+ std::to_string(args.stable_frames)
		+ " consecutive frames, timeout "
		+ std::to_string(static_cast<int>(args.track_timeout_s)) + "s)");

	const auto t0 = std::chrono::steady_clock::now();
	int streak = 0;
	while (!g_stop.load()) {
		const double elapsed = std::chrono::duration<double>(
			std::chrono::steady_clock::now() - t0).count();
		if (elapsed >= args.track_timeout_s) {
			log_error("chase_hunt", "Timed out waiting for dual track");
			return false;
		}

		TrackingFrame frame;
		if (tracker.get_tracking_frame(frame) && frame.both_valid()
			&& frame.distance_mm > 0.0f) {
			++streak;
			if (args.verbose && (streak == 1 || streak % 10 == 0)) {
				std::cout << "  track streak=" << streak
					<< " dist=" << frame.distance_mm << " mm\n";
			}
			if (streak >= args.stable_frames) {
				out_frame = frame;
				log_info("chase_hunt",
					"Dual track OK: distance_mm="
					+ std::to_string(frame.distance_mm)
					+ " bearing_deg=" + std::to_string(frame.bearing_deg)
					+ " closing_mm_s="
					+ std::to_string(frame.closing_speed_mm_s));
				return true;
			}
		} else {
			streak = 0;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(5));
	}
	return false;
}

// --- Phase 4/5: MotionPlanner flee + encoder score -------------------------

// Accept if |err| ≤ max(15 mm, 10% of |request|) — matches lab MAE ~13 mm.
bool travel_within_tolerance(float request_mm, float actual_mm) {
	const float err = std::fabs(actual_mm - request_mm);
	const float tol = std::max(15.0f, 0.10f * std::fabs(request_mm));
	return err <= tol;
}

bool execute_flee(PreyMotor& motor, MotionPlanner& planner, const Args& args,
	float& actual_mm, float& plan_mm, float& actual_duration_s)
{
	const ChainMovePlan plan = MotionPlanner::runtime_plan_distance_mm_in_time(
		motor, args.distance_mm, args.duration_ms, args.max_accel_mps2);
	plan_mm = plan.expected_distance_mm;

	std::cout << "=== Flee plan ===\n"
		<< "  request=" << args.distance_mm << " mm in "
		<< args.duration_ms << " ms  accel=" << args.max_accel_mps2 << "\n"
		<< "  feasible=" << (plan.feasible ? "yes" : "no")
		<< "  expected=" << plan.expected_distance_mm << " mm"
		<< "  peak=" << plan.peak_turns_s << " turns/s"
		<< "  duration_s=" << plan.duration_s << "\n";

	if (!plan.feasible) {
		log_error("chase_hunt", "MotionPlanner plan not feasible");
		return false;
	}
	if (args.dry_run) {
		log_info("chase_hunt", "Dry-run — skipping motor execute");
		actual_mm = 0.0f;
		actual_duration_s = 0.0f;
		return true;
	}

	const float pos0 = motor.read_position_turns();
	const auto t0 = std::chrono::steady_clock::now();
	if (!planner.start_plan(plan)) {
		log_error("chase_hunt", "start_plan failed");
		return false;
	}

	bool ok = false;
	while (!g_stop.load()) {
		const MoveTick tick = planner.tick(motor);
		if (tick == MoveTick::Done) {
			ok = true;
			break;
		}
		if (tick == MoveTick::Cancelled || tick == MoveTick::Idle) {
			break;
		}
		// Why: chase control loop cadence (~50 Hz watchdog feed).
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}

	actual_duration_s = std::chrono::duration<float>(
		std::chrono::steady_clock::now() - t0).count();
	actual_mm = motor.turns_to_chain_mm(motor.read_position_turns() - pos0);
	motor.stop();
	return ok && !g_stop.load();
}

} // namespace

int main(int argc, char** argv) {
	Args args;
	if (!parse_args(argc, argv, args)) {
		print_usage();
		return 1;
	}

	Logger::instance().set_level(args.verbose ? LogLevel::Debug : LogLevel::Info);
	std::signal(SIGINT, signal_handler);
	std::signal(SIGTERM, signal_handler);

	ArenaExperimentConfig arena_cfg;
	const std::string arena_path =
		resolve_arena_config_path(argv[0], args.arena_config);
	if (!load_arena_experiment_config(arena_path, arena_cfg)) {
		log_error("chase_hunt", "Failed to load arena config: " + arena_path);
		return 1;
	}
	if (arena_cfg.motor.chain_mm_per_motor_turn <= 0.0f) {
		log_error("chase_hunt",
			"motor.chain_mm_per_motor_turn must be set in arena config");
		return 1;
	}

	Pylon::PylonInitialize();
	int exit_code = 1;
	try {
		// --- Camera connected / on ---
		Pylon::CBaslerUniversalInstantCamera camera(
			Pylon::CTlFactory::GetInstance().CreateFirstDevice());
		CameraSettings camera_settings;
		if (!open_camera(camera, args, argv, camera_settings)) {
			Pylon::PylonTerminate();
			return 1;
		}

		std::optional<CameraCalib> calib;
		if (!args.disable_calib) {
			const std::string calib_path =
				resolve_calib_path(argv[0], args.calib_path);
			calib = load_camera_calib(calib_path,
				cv::Size(camera_settings.width, camera_settings.height));
			if (!calib) {
				log_info("chase_hunt",
					"No calib.npz — continuing without undistort "
					"(pass --calib or --no-calib)");
			}
		}

		CameraTrackingService::Options track_opts;
		track_opts.warmup_frames = static_cast<int>(args.warmup_s * FPS);
		track_opts.gsd_mm_px = args.gsd_mm_px;
		track_opts.fps = FPS;
		track_opts.calib = calib;
		track_opts.mask = arena_cfg.vision.mask;
		track_opts.vision = arena_cfg.vision;
		track_opts.enable_display = false;
		CameraTrackingService tracker(track_opts);

		camera.RegisterImageEventHandler(&tracker,
			Pylon::RegistrationMode_Append, Pylon::Cleanup_None);
		camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly,
			Pylon::GrabLoop_ProvidedByInstantCamera);
		if (!camera.IsGrabbing()) {
			log_error("chase_hunt", "Camera failed to start grabbing");
			Pylon::PylonTerminate();
			return 1;
		}
		log_info("chase_hunt", "Camera grabbing — warmup "
			+ std::to_string(static_cast<int>(args.warmup_s))
			+ "s (keep arena empty)");

		// Let MOG2 learn background before requiring tracks.
		for (int ms = 0; ms < static_cast<int>(args.warmup_s * 1000.0)
			&& !g_stop.load(); ms += 50) {
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
		}
		if (g_stop.load()) {
			camera.StopGrabbing();
			Pylon::PylonTerminate();
			return 1;
		}
		log_info("chase_hunt",
			"Warmup done — introduce ferret + prey into view");

		// --- Motor connected / on ---
		PreyMotor motor(prey_motor_from_config(arena_cfg.motor));
		if (!connect_motor(arena_cfg, motor)) {
			camera.StopGrabbing();
			camera.DeregisterImageEventHandler(&tracker);
			Pylon::PylonTerminate();
			return 1;
		}

		// --- Dual track + distance ---
		TrackingFrame scene;
		if (!wait_for_dual_track(tracker, args, scene)) {
			motor.stop();
			camera.StopGrabbing();
			camera.DeregisterImageEventHandler(&tracker);
			Pylon::PylonTerminate();
			return 1;
		}

		std::cout << "=== Scene at flee trigger ===\n"
			<< "  ferret=(" << scene.ferret.state.pos_mm.x << ", "
			<< scene.ferret.state.pos_mm.y << ") mm\n"
			<< "  prey=(" << scene.prey.state.pos_mm.x << ", "
			<< scene.prey.state.pos_mm.y << ") mm\n"
			<< "  vision_distance=" << scene.distance_mm << " mm\n";

		// --- MotionPlanner hunt flee ---
		MotionPlanner planner;
		g_planner = &planner;
		float actual_mm = 0.0f;
		float plan_mm = 0.0f;
		float actual_duration_s = 0.0f;
		const bool move_ok = execute_flee(motor, planner, args,
			actual_mm, plan_mm, actual_duration_s);
		g_planner = nullptr;

		// Optional post-move vision sample (non-fatal if lost during move).
		TrackingFrame after;
		if (tracker.get_tracking_frame(after) && after.both_valid()) {
			std::cout << "=== Scene after flee ===\n"
				<< "  vision_distance=" << after.distance_mm << " mm"
				<< " (was " << scene.distance_mm << " mm)\n";
		}

		const float err = actual_mm - args.distance_mm;
		const bool dist_ok = args.dry_run
			|| travel_within_tolerance(args.distance_mm, actual_mm);

		std::cout << "=== Result ===\n"
			<< "  move_ok=" << (move_ok ? "yes" : "no")
			<< "  encoder=" << actual_mm << " mm"
			<< "  request=" << args.distance_mm << " mm"
			<< "  err=" << err << " mm"
			<< "  wall_s=" << actual_duration_s << "\n"
			<< "  distance_pass=" << (dist_ok ? "yes" : "no") << "\n";

		exit_code = (move_ok && dist_ok) ? 0 : 1;

		motor.stop();
		camera.StopGrabbing();
		camera.DeregisterImageEventHandler(&tracker);
	} catch (const Pylon::GenericException& e) {
		log_error("chase_hunt",
			std::string("Pylon error: ") + e.GetDescription());
		exit_code = 1;
	} catch (const std::exception& e) {
		log_error("chase_hunt", e.what());
		exit_code = 1;
	}

	Pylon::PylonTerminate();
	return exit_code;
}
