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

#include "camera_config.h"
#include "camera_settings.h"
#include "ferret_tracker.h"
#include "display.h"
#include "logger.h"

using namespace Pylon;

static std::atomic<bool> g_running{true};

void signal_handler(int) { g_running.store(false); }

struct AppOptions {
	bool enable_display = false;
	bool verbose = false;
	std::string log_file;
	std::string camera_config;
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

int main(int argc, char** argv) {
	AppOptions opts;
	// Logger uses default Info until init_logger runs after arg parse.
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

		FerretTracker tracker(opts.enable_display);
		camera.RegisterImageEventHandler(&tracker,
			RegistrationMode_Append, Cleanup_None);

		if (opts.enable_display) {
			display.start(&tracker, &g_running);
			display_started = true;
		}

		log_info("main", "Warming up background model — keep arena empty for 30s");
		camera.StartGrabbing(GrabStrategy_LatestImageOnly,
			GrabLoop_ProvidedByInstantCamera);

		while (g_running.load()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(5));

			if (!tracker.ferret.valid || !tracker.prey.valid) continue;

			// Telemetry: raw stdout without logger prefix for piping/scripts.
			float dx = tracker.ferret.pos_mm.x - tracker.prey.pos_mm.x;
			float dy = tracker.ferret.pos_mm.y - tracker.prey.pos_mm.y;
			float distance_mm = std::sqrt(dx * dx + dy * dy);

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

	PylonTerminate();
	return 0;
}
