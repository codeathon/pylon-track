// Suite 3 — mounting height validation (operator-in-the-loop).
//
// Camera height cannot be swept automatically: the operator mounts the camera
// at a candidate height, runs this tool, and repeats per height. The tool:
//   1. Rescales GSD for the entered height (production constant assumes 1.2m)
//      so mm measurements stay correct.
//   2. Saves annotated stills (~1 Hz): contours, bounding boxes, blob px²
//      areas — to judge pixel resolution against the 200 px² tracking floor.
//   3. Runs the same measurement loop as suite 2 so latency and distance
//      accuracy can be compared across heights from the CSVs.
//
// Usage:
//   test_mount_height --height-cm 120 [--duration 30] [--warmup-secs 30]
//       [--camera-config path] [--output tests/output] [--verbose]

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

#include <atomic>
#include <csignal>
#include <cstring>
#include <iostream>
#include <string>

#include "camera/camera_config.h"
#include "camera/camera_settings.h"
#include "tracker/ferret_tracker.h"
#include "log/logger.h"
#include "measurement_run.h"
#include "test_session.h"

// Production optics baseline: GSD_MM_PX was measured at this mount height.
static constexpr double REFERENCE_HEIGHT_CM = 120.0;

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running.store(false); }

struct HeightArgs {
	MeasurementOptions opts;
	std::string camera_config;
	std::string output_base = "tests/output";
};

static bool parse_height_args(int argc, char** argv, HeightArgs& args) {
	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "--height-cm") == 0 && i + 1 < argc) {
			args.opts.height_cm = std::stod(argv[++i]);
		} else if (std::strcmp(argv[i], "--duration") == 0 && i + 1 < argc) {
			args.opts.duration_s = std::stod(argv[++i]);
		} else if (std::strcmp(argv[i], "--warmup-secs") == 0 && i + 1 < argc) {
			args.opts.warmup_s = std::stod(argv[++i]);
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
	if (args.opts.height_cm <= 0.0) {
		std::cerr << "Usage: test_mount_height --height-cm <h> "
			"[--duration <s>] [--warmup-secs <s>]\n";
		return false;
	}
	return true;
}

int main(int argc, char** argv) {
	HeightArgs args;
	if (!parse_height_args(argc, argv, args)) {
		return 1;
	}
	std::signal(SIGINT, signal_handler);
	std::signal(SIGTERM, signal_handler);

	// GSD scales linearly with distance for a fixed lens (pinhole model).
	args.opts.gsd_mm_px = static_cast<float>(
		GSD_MM_PX * args.opts.height_cm / REFERENCE_HEIGHT_CM);
	// ~1 still per second at the production frame rate.
	args.opts.annotate_every_n = static_cast<int>(FPS);

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
		log_info("height", "Camera: "
			+ std::string(camera.GetDeviceInfo().GetModelName()));
		configure_camera(camera, settings);

		const std::string label =
			"h" + csv_num(args.opts.height_cm, 0) + "cm";
		args.opts.session_dir =
			make_session_dir(args.output_base, "mount_height", label);
		log_info("height", "Height " + csv_num(args.opts.height_cm, 1)
			+ "cm -> GSD " + csv_num(args.opts.gsd_mm_px, 4) + " mm/px");

		exit_code = run_measurement(camera, args.opts, g_running);
	} catch (const Pylon::GenericException& e) {
		log_error("height", std::string("Pylon error: ") + e.GetDescription());
		exit_code = 1;
	} catch (const std::exception& e) {
		log_error("height", e.what());
		exit_code = 1;
	}
	Pylon::PylonTerminate();
	return exit_code;
}
