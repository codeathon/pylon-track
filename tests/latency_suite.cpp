// Suite 2 — two-object latency benchmark.
//
// Runs the PRODUCTION tracking pipeline (MOG2 + Kalman, FerretTracker) on two
// moving objects and records, per frame: both speeds, both centroids (px and
// mm), the distance between objects, and the grab-to-distance latency.
// Vary the physical object speeds between runs; compare frames.csv rows.
//
// Usage:
//   test_latency [--duration 30] [--warmup-secs 30] [--gsd 1.035]
//       [--camera-config path] [--output tests/output] [--verbose]
//
// Protocol: keep the arena empty during the warmup window, then introduce the
// two moving objects for the measured duration.

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

#include <atomic>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>

#include "camera_config.h"
#include "camera_settings.h"
#include "logger.h"
#include "measurement_run.h"
#include "test_session.h"

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running.store(false); }

struct LatencyArgs {
	MeasurementOptions opts;
	std::string camera_config;
	std::string output_base = "tests/output";
};

static bool parse_latency_args(int argc, char** argv, LatencyArgs& args) {
	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
			args.opts.duration_s = std::stod(argv[++i]);
		} else if (std::strcmp(argv[i], "--warmup-secs") == 0 && i + 1 < argc) {
			args.opts.warmup_s = std::stod(argv[++i]);
		} else if (std::strcmp(argv[i], "--gsd") == 0 && i + 1 < argc) {
			args.opts.gsd_mm_px = std::stof(argv[++i]);
		} else if (std::strcmp(argv[i], "--camera-config") == 0 && i + 1 < argc) {
			args.camera_config = argv[++i];
		} else if (std::strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
			args.output_base = argv[++i];
		} else if (std::strcmp(argv[i], "--verbose") == 0) {
			Logger::instance().set_level(LogLevel::Debug);
		} else {
			std::cerr << "Unknown argument: " << argv[i] << '\n';
			return false;
		}
	}
	return true;
}

int main(int argc, char** argv) {
	LatencyArgs args;
	if (!parse_latency_args(argc, argv, args)) {
		return 1;
	}
	std::signal(SIGINT, signal_handler);
	std::signal(SIGTERM, signal_handler);

	CameraSettings settings;
	const std::string config_path =
		resolve_camera_config_path(argv[0], args.camera_config);
	if (!load_camera_config(config_path, settings)) {
		return 1;
	}

	Pylon::PylonInitialize();
	int exit_code = 0;
	try {
		Pylon::CBaslerUniversalInstantCamera camera(
			Pylon::CTlFactory::GetInstance().CreateFirstDevice());
		log_info("latency", "Camera: "
			+ std::string(camera.GetDeviceInfo().GetModelName()));
		configure_camera(camera, settings);

		args.opts.session_dir =
			make_session_dir(args.output_base, "latency", "");
		exit_code = run_measurement(camera, args.opts, g_running);
	} catch (const Pylon::GenericException& e) {
		log_error("latency", std::string("Pylon error: ") + e.GetDescription());
		exit_code = 1;
	} catch (const std::exception& e) {
		log_error("latency", e.what());
		exit_code = 1;
	}
	Pylon::PylonTerminate();
	return exit_code;
}
