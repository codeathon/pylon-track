#pragma once

#include <string>

struct CameraSettings {
	std::string pixel_format = "Mono8";
	int width = 1920;
	int height = 960;
	int offset_x = 0;
	int offset_y = 120;
	bool exposure_auto = false;
	double exposure_time_us = 5000.0;
	// Standard (19 µs–10 s) or UltraShort (1–14 µs); JSON alias "Common" ok.
	std::string exposure_time_mode = "Standard";
	bool gain_auto = false;
	double gain_db = 6.0;
	bool frame_rate_enable = true;
	double frame_rate_fps = 200.0;
	std::string trigger_mode = "Off";
	std::string device_link_throughput_limit = "Off";
	// When > 0 and mode is On, caps USB throughput (bytes/s set from Mbps).
	double device_link_throughput_mbps = 0.0;
	int black_level = 0;
	double gamma = 1.0;
	int binning_horizontal = 1;
	int binning_vertical = 1;
	// Region1/FPGA or Sensor — ace 2 USB; JSON alias "FPGA" maps to Region1.
	std::string binning_selector = "Sensor";
	double scaling_horizontal = 1.0;
	bool reverse_x = false;
	bool reverse_y = false;
};

// Load camera settings from JSON; returns false and logs on failure.
bool load_camera_config(const std::string& path, CameraSettings& out);

// Resolve config path: CLI > env > next to executable > src/camera/camera_config.json.
std::string resolve_camera_config_path(const char* argv0,
	const std::string& cli_path);
