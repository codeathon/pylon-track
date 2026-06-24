#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <cstring>
#include <exception>
#include <string>
#include <filesystem>
#include <cmath>

#include "camera/camera_config.h"
#include "camera/camera_settings.h"
#include "camera/camera_calib.h"
#include "experiment/arena_config.h"
#include "experiment/session_recorder.h"
#include "experiment/trial_state.h"
#include "tracker/ferret_tracker.h"
#include "tracker/display.h"
#include "log/logger.h"

using namespace Pylon;
namespace fs = std::filesystem;

static std::atomic<bool> g_running{true};

void signal_handler(int) { g_running.store(false); }

struct AppOptions {
	bool enable_display = false;
	bool verbose = false;
	std::string log_file;
	std::string camera_config;
	std::string calib_path;
	bool disable_calib = false;
	std::string experiment_config;
	std::string session_dir = "sessions";
};

static bool parse_args(int argc, char** argv, AppOptions& opts) {
	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "--display") == 0) {
			opts.enable_display = true;
		} else if (std::strcmp(argv[i], "--verbose") == 0) {
			opts.verbose = true;
		} else if (std::strcmp(argv[i], "--log-file") == 0) {
			if (i + 1 >= argc) {
				std::cerr << "ERROR: --log-file requires a path argument\n";
				return false;
			}
			opts.log_file = argv[++i];
		} else if (std::strcmp(argv[i], "--camera-config") == 0) {
			if (i + 1 >= argc) {
				std::cerr << "ERROR: --camera-config requires a path argument\n";
				return false;
			}
			opts.camera_config = argv[++i];
		} else if (std::strcmp(argv[i], "--calib") == 0) {
			if (i + 1 >= argc) {
				std::cerr << "ERROR: --calib requires a path argument\n";
				return false;
			}
			opts.calib_path = argv[++i];
		} else if (std::strcmp(argv[i], "--no-calib") == 0) {
			opts.disable_calib = true;
		} else if (std::strcmp(argv[i], "--experiment") == 0) {
			if (i + 1 >= argc) {
				std::cerr << "ERROR: --experiment requires a path argument\n";
				return false;
			}
			opts.experiment_config = argv[++i];
		} else if (std::strcmp(argv[i], "--session") == 0) {
			if (i + 1 >= argc) {
				std::cerr << "ERROR: --session requires a path argument\n";
				return false;
			}
			opts.session_dir = argv[++i];
		} else {
			std::cerr << "ERROR: Unknown argument: " << argv[i] << '\n';
			return false;
		}
	}
	return true;
}

static void init_logger(const AppOptions& opts) {
	Logger& logger = Logger::instance();
	logger.set_level(opts.verbose ? LogLevel::Debug : LogLevel::Info);
	if (!opts.log_file.empty()) {
		logger.set_log_file(opts.log_file);
		log_info("main", "Logging to file: " + opts.log_file);
	}
}

static int64_t host_time_ns() {
	return std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count();
}

// Resolve arena_experiment.json beside the executable or under config/.
static std::string resolve_experiment_config_path(const char* argv0,
	const std::string& user_path)
{
	if (!user_path.empty()) {
		return user_path;
	}
	const fs::path exe_dir = fs::path(argv0).parent_path();
	const fs::path candidates[] = {
		exe_dir / "arena_experiment.json",
		exe_dir / "config" / "arena_experiment.json",
		fs::path("config") / "arena_experiment.json",
	};
	for (const auto& path : candidates) {
		if (fs::exists(path)) {
			return path.string();
		}
	}
	return {};
}

static void operator_input_thread(TrialStateMachine* fsm, SessionRecorder* recorder) {
	while (g_running.load()) {
		char key = 0;
		if (!(std::cin >> key)) {
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			continue;
		}
		const int64_t ts = host_time_ns();
		if (key == 's') {
			if (fsm->start_trial()) {
				log_info("experiment", "Trial started (Running)");
				if (recorder && recorder->is_open()) {
					recorder->log_event("trial_start", ts, fsm->phase());
				}
			} else {
				log_info("experiment",
					"Cannot start trial — wait for Armed (warmup complete), then press s");
			}
		} else if (key == 'e') {
			if (fsm->end_trial()) {
				log_info("experiment", "Trial ended");
				if (recorder && recorder->is_open()) {
					recorder->log_event("trial_end", ts, fsm->phase());
				}
			}
		} else if (key == 'r') {
			if (fsm->reset_trial()) {
				log_info("experiment", "Trial reset — Armed, ready for next start");
				if (recorder && recorder->is_open()) {
					recorder->log_event("trial_reset", ts, fsm->phase());
				}
			}
		}
	}
}

int main(int argc, char** argv) {
	AppOptions opts;
	if (!parse_args(argc, argv, opts)) {
		return 1;
	}
	init_logger(opts);

	std::signal(SIGINT,  signal_handler);
	std::signal(SIGTERM, signal_handler);

	if (opts.enable_display && std::getenv("DISPLAY") == nullptr) {
		log_error("main", "--display requires a graphical session (DISPLAY is unset)");
		return 1;
	}

	const bool experiment_mode = !opts.experiment_config.empty();

	ArenaExperimentConfig arena_cfg;
	SessionRecorder session_recorder;
	TrialStateMachine trial_fsm;
	std::thread operator_thread;

	if (experiment_mode) {
		const std::string cfg_path = resolve_experiment_config_path(
			argv[0], opts.experiment_config);
		if (cfg_path.empty()) {
			log_error("experiment",
				"No arena config found — pass --experiment <path> or add config/arena_experiment.json");
			return 1;
		}
		if (!load_arena_experiment_config(cfg_path, arena_cfg)) {
			return 1;
		}
		try {
			session_recorder.open_session(opts.session_dir, "arena_experiment");
		} catch (const std::exception& e) {
			log_error("experiment", std::string("Session recorder failed: ") + e.what());
			return 1;
		}
		trial_fsm.begin_warmup();
		operator_thread = std::thread(operator_input_thread, &trial_fsm, &session_recorder);
		log_info("experiment",
			"Experiment mode — warmup 30s, then s=start e=end r=reset");
	}

	PylonInitialize();

	DisplayThread display;
	bool display_started = false;

	try {
		CBaslerUniversalInstantCamera camera(
			CTlFactory::GetInstance().CreateFirstDevice());

		log_info("main", "Camera: " + std::string(camera.GetDeviceInfo().GetModelName()));
		if (opts.enable_display) {
			log_info("main",
				"Live display enabled (helper thread). Press q or ESC in window to quit.");
		}

		const std::string config_path =
			resolve_camera_config_path(argv[0], opts.camera_config);
		log_info("camera", "Loading camera config: " + config_path);

		CameraSettings camera_settings;
		if (!load_camera_config(config_path, camera_settings)) {
			PylonTerminate();
			return 1;
		}
		try {
			configure_camera(camera, camera_settings);
		} catch (const std::exception& e) {
			log_error("camera", std::string("Camera configuration failed: ") + e.what());
			PylonTerminate();
			return 1;
		}

		std::optional<CameraCalib> calib;
		if (!opts.disable_calib) {
			const std::string calib_path =
				resolve_calib_path(argv[0], opts.calib_path);
			if (!calib_path.empty()) {
				log_info("calib", "Loading lens calibration: " + calib_path);
				calib = load_camera_calib(calib_path,
					cv::Size(camera_settings.width, camera_settings.height));
				if (!calib) {
					PylonTerminate();
					return 1;
				}
			} else {
				log_info("calib",
					"No calib.npz found — running without lens undistort "
					"(use --calib or place calib.npz beside the executable)");
			}
		}

		FerretTracker tracker(opts.enable_display, WARMUP_FRAMES, GSD_MM_PX, calib);
		if (experiment_mode) {
			tracker.set_experiment_hooks(&trial_fsm, &session_recorder);
		}
		camera.RegisterImageEventHandler(&tracker,
			RegistrationMode_Append, Cleanup_None);

		if (opts.enable_display) {
			display.start(&tracker, &g_running);
			display_started = true;
		}

		log_info("main", "Warming up background model — keep arena empty for 30s");
		camera.StartGrabbing(GrabStrategy_LatestImageOnly,
			GrabLoop_ProvidedByInstantCamera);

		TrialPhase last_phase = TrialPhase::Idle;
		while (g_running.load()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(5));

			if (experiment_mode) {
				const TrialPhase phase = trial_fsm.phase();
				if (phase != last_phase) {
					log_info("experiment",
						std::string("Phase: ") + trial_phase_name(phase));
					last_phase = phase;
				}
				continue;
			}

			if (!tracker.ferret.valid || !tracker.prey.valid) {
				continue;
			}

			// Telemetry: raw stdout without logger prefix for piping/scripts.
			const float dx = tracker.ferret.pos_mm.x - tracker.prey.pos_mm.x;
			const float dy = tracker.ferret.pos_mm.y - tracker.prey.pos_mm.y;
			const float distance_mm = std::sqrt(dx * dx + dy * dy);

			std::printf(
				"Ferret: (%.0f, %.0f)mm  %.0fmm/s  %.0fdeg  |  "
				"Prey: (%.0f, %.0f)mm  %.0fmm/s  %.0fdeg  |  "
				"Dist: %.0fmm\n",
				tracker.ferret.pos_mm.x, tracker.ferret.pos_mm.y,
				tracker.ferret.speed_mm_s,
				tracker.ferret.direction_deg,
				tracker.prey.pos_mm.x, tracker.prey.pos_mm.y,
				tracker.prey.speed_mm_s,
				tracker.prey.direction_deg,
				distance_mm);
		}

		camera.StopGrabbing();
		if (display_started) {
			display.stop();
			display_started = false;
		}
		log_info("main", "Stopped");

	} catch (const GenericException& e) {
		if (display_started) {
			display.stop();
		}
		log_error("main", std::string("Pylon error: ") + e.GetDescription());
		PylonTerminate();
		return 1;
	}

	g_running.store(false);
	if (operator_thread.joinable()) {
		operator_thread.join();
	}

	PylonTerminate();
	return 0;
}
