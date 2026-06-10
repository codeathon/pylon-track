#include "camera_config.h"
#include "logger.h"

#include <fstream>
#include <nlohmann/json.hpp>
#include <sstream>
#include <unistd.h>
#include <filesystem>

namespace fs = std::filesystem;

namespace {

bool file_exists(const std::string& path) {
	return !path.empty() && fs::is_regular_file(path);
}

std::string exe_directory(const char* argv0) {
	if (argv0 == nullptr || argv0[0] == '\0') {
		return {};
	}
	std::error_code ec;
	const fs::path exe = fs::weakly_canonical(fs::path(argv0), ec);
	if (ec) {
		return {};
	}
	return exe.parent_path().string();
}

template<typename T>
T require_field(const nlohmann::json& j, const char* key) {
	if (!j.contains(key)) {
		throw std::runtime_error(std::string("Missing required field: ") + key);
	}
	return j.at(key).get<T>();
}

} // namespace

std::string resolve_camera_config_path(const char* argv0,
	const std::string& cli_path)
{
	if (!cli_path.empty()) {
		return cli_path;
	}

	const char* env_path = std::getenv("PYLON_CAMERA_CONFIG");
	if (env_path != nullptr && env_path[0] != '\0') {
		return env_path;
	}

	const std::string beside_exe = exe_directory(argv0) + "/camera_config.json";
	if (file_exists(beside_exe)) {
		return beside_exe;
	}

	const std::string src_relative = "src/camera_config.json";
	if (file_exists(src_relative)) {
		return src_relative;
	}

	// Fallback for cmake copy target path when cwd is build/.
	if (file_exists("bin/camera_config.json")) {
		return "bin/camera_config.json";
	}

	return src_relative;
}

bool load_camera_config(const std::string& path, CameraSettings& out) {
	std::ifstream file(path);
	if (!file.is_open()) {
		log_error("camera", "Cannot open camera config: " + path);
		return false;
	}

	nlohmann::json j;
	try {
		file >> j;
	} catch (const nlohmann::json::exception& e) {
		log_error("camera", std::string("JSON parse error in ") + path + ": " + e.what());
		return false;
	}

	try {
		out.pixel_format = require_field<std::string>(j, "pixel_format");
		out.width = require_field<int>(j, "width");
		out.height = require_field<int>(j, "height");
		out.offset_x = require_field<int>(j, "offset_x");
		out.offset_y = require_field<int>(j, "offset_y");
		out.exposure_auto = require_field<bool>(j, "exposure_auto");
		out.exposure_time_us = require_field<double>(j, "exposure_time_us");
		out.gain_auto = require_field<bool>(j, "gain_auto");
		out.gain_db = require_field<double>(j, "gain_db");
		out.frame_rate_enable = require_field<bool>(j, "frame_rate_enable");
		out.frame_rate_fps = require_field<double>(j, "frame_rate_fps");
		out.trigger_mode = require_field<std::string>(j, "trigger_mode");
		out.device_link_throughput_limit =
			require_field<std::string>(j, "device_link_throughput_limit");
	} catch (const std::exception& e) {
		log_error("camera", std::string("Invalid camera config in ") + path + ": " + e.what());
		return false;
	}

	return true;
}

void configure_camera(CBaslerUniversalInstantCamera& cam, const CameraSettings& s) {
	cam.Open();

	if (s.pixel_format != "Mono8") {
		throw std::runtime_error("Unsupported pixel_format: " + s.pixel_format + " (only Mono8)");
	}
	cam.PixelFormat.SetValue(PixelFormat_Mono8);

	cam.Width.SetValue(s.width);
	cam.Height.SetValue(s.height);
	cam.OffsetX.SetValue(s.offset_x);
	cam.OffsetY.SetValue(s.offset_y);

	cam.ExposureAuto.SetValue(s.exposure_auto ? ExposureAuto_Continuous : ExposureAuto_Off);
	if (!s.exposure_auto) {
		cam.ExposureTime.SetValue(s.exposure_time_us);
	}

	cam.GainAuto.SetValue(s.gain_auto ? GainAuto_Continuous : GainAuto_Off);
	if (!s.gain_auto) {
		cam.Gain.SetValue(s.gain_db);
	}

	cam.AcquisitionFrameRateEnable.SetValue(s.frame_rate_enable);
	if (s.frame_rate_enable) {
		cam.AcquisitionFrameRate.SetValue(s.frame_rate_fps);
	}

	if (s.trigger_mode == "Off") {
		cam.TriggerMode.SetValue(TriggerMode_Off);
	} else {
		throw std::runtime_error("Unsupported trigger_mode: " + s.trigger_mode);
	}

	if (s.device_link_throughput_limit == "Off") {
		cam.DeviceLinkThroughputLimitMode.SetValue(DeviceLinkThroughputLimitMode_Off);
	} else {
		throw std::runtime_error(
			"Unsupported device_link_throughput_limit: " + s.device_link_throughput_limit);
	}

	std::ostringstream oss;
	oss << "Applied " << s.pixel_format << " " << s.width << "x" << s.height
		<< " offset=(" << s.offset_x << "," << s.offset_y << ")"
		<< " exposure=" << s.exposure_time_us << "us"
		<< " gain=" << s.gain_db << "dB"
		<< " fps=" << s.frame_rate_fps;
	log_info("camera", oss.str());
}
