#pragma once

#include <string>

struct CameraSettings {
	std::string pixel_format = "Mono8";
	int width = 1920;
	int height = 960;
	int offset_x = 0;
	int offset_y = 120;
	bool exposure_auto = false;
	double exposure_time_us = 2000.0;
	bool gain_auto = false;
	double gain_db = 6.0;
	bool frame_rate_enable = true;
	double frame_rate_fps = 200.0;
	std::string trigger_mode = "Off";
	std::string device_link_throughput_limit = "Off";
};

// Load camera settings from JSON; returns false and logs on failure.
bool load_camera_config(const std::string& path, CameraSettings& out);

// Resolve config path: CLI > env > next to executable > src/camera_config.json.
std::string resolve_camera_config_path(const char* argv0,
	const std::string& cli_path);
