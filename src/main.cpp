#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <iostream>
#include <thread>
#include <chrono>
#include <csignal>
#include <atomic>
#include <cstring>

#include "camera_config.h"
#include "ferret_tracker.h"
#include "display.h"

using namespace Pylon;

static std::atomic<bool> g_running{true};

void signal_handler(int) { g_running.store(false); }

static bool parse_display_flag(int argc, char** argv) {
	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "--display") == 0) {
			return true;
		}
	}
	return false;
}

int main(int argc, char** argv) {
	const bool enable_display = parse_display_flag(argc, argv);

	std::signal(SIGINT,  signal_handler);
	std::signal(SIGTERM, signal_handler);

	if (enable_display && std::getenv("DISPLAY") == nullptr) {
		std::cerr << "ERROR: --display requires a graphical session (DISPLAY is unset).\n";
		return 1;
	}

	PylonInitialize();

	DisplayThread display;
	bool display_started = false;

	try {
		CBaslerUniversalInstantCamera camera(
			CTlFactory::GetInstance().CreateFirstDevice());

		std::cout << "Camera: " << camera.GetDeviceInfo().GetModelName() << "\n";
		if (enable_display) {
			std::cout << "Live display enabled (helper thread). Press q or ESC in window to quit.\n";
		}

		configure_camera(camera);

		FerretTracker tracker(enable_display);
		camera.RegisterImageEventHandler(&tracker,
			RegistrationMode_Append, Cleanup_None);

		if (enable_display) {
			display.start(&tracker, &g_running);
			display_started = true;
		}

		std::cout << "Warming up background model — keep arena empty for 30s...\n";
		camera.StartGrabbing(GrabStrategy_LatestImageOnly,
			GrabLoop_ProvidedByInstantCamera);

		while (g_running.load()) {
			std::this_thread::sleep_for(std::chrono::milliseconds(5));

			if (!tracker.ferret.valid || !tracker.prey.valid) continue;

			// Inter-animal distance
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
		std::cout << "Stopped.\n";

	} catch (const GenericException& e) {
		if (display_started) {
			display.stop();
		}
		std::cerr << "Pylon error: " << e.GetDescription() << "\n";
		PylonTerminate();
		return 1;
	}

	PylonTerminate();
	return 0;
}
