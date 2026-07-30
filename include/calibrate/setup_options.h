#pragma once

#include <string>

// Which one-time setup step to run (All = bundled default).
enum class SetupStep {
	All,
	Arena,
	Camera,
	ODrive,
	LabJack
};

SetupStep parse_setup_step(const std::string& name);

struct SetupOptions {
	const char* argv0 = nullptr;
	std::string arena_config_path;
	std::string camera_config_path;
	std::string calib_path;
	std::string calib_frames_dir = "calib_frames";
	bool verbose = false;
	bool skip_interactive = false;
	bool display = false;
	SetupStep only = SetupStep::All;
	// ODrive setup test-spin duration (seconds).
	float spin_seconds = 10.0f;
};
