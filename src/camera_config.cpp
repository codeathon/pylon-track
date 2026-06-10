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

void apply_binning_selector(CBaslerUniversalInstantCamera& cam,
	const std::string& selector)
{
	if (selector == "FPGA") {
		cam.BinningSelector.SetValue(BinningSelector_FPGA);
	} else if (selector == "Sensor") {
		cam.BinningSelector.SetValue(BinningSelector_Sensor);
	} else {
		throw std::runtime_error("Unsupported binning_selector: " + selector);
	}
}

void apply_exposure_time_mode(CBaslerUniversalInstantCamera& cam,
	const std::string& mode)
{
	if (mode == "Common") {
		cam.BslExposureTimeMode.SetValue(BslExposureTimeMode_Common);
	} else if (mode == "UltraShort") {
		cam.BslExposureTimeMode.SetValue(BslExposureTimeMode_UltraShort);
	} else {
		throw std::runtime_error("Unsupported exposure_time_mode: " + mode);
	}
}

void apply_throughput_limit(CBaslerUniversalInstantCamera& cam,
	const CameraSettings& s)
{
	if (s.device_link_throughput_limit == "Off") {
		cam.DeviceLinkThroughputLimitMode.SetValue(
			DeviceLinkThroughputLimitMode_Off);
		return;
	}
	if (s.device_link_throughput_limit != "On") {
		throw std::runtime_error(
			"Unsupported device_link_throughput_limit: "
			+ s.device_link_throughput_limit);
	}
	if (s.device_link_throughput_mbps <= 0.0) {
		throw std::runtime_error(
			"device_link_throughput_mbps required when throughput limit is On");
	}
	// GenICam limit is bytes/s; sweep specs use Mbps for readability.
	const int64_t bytes_per_s = static_cast<int64_t>(
		s.device_link_throughput_mbps * 1e6 / 8.0);
	cam.DeviceLinkThroughputLimitMode.SetValue(DeviceLinkThroughputLimitMode_On);
	cam.DeviceLinkThroughputLimit.SetValue(bytes_per_s);
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
		// Optional fields — older camera_config.json copies still load.
		out.exposure_time_mode = j.value("exposure_time_mode", "Common");
		out.device_link_throughput_mbps =
			j.value("device_link_throughput_mbps", 0.0);
		out.black_level = j.value("black_level", 0);
		out.gamma = j.value("gamma", 1.0);
		out.binning_horizontal = j.value("binning_horizontal", 1);
		out.binning_vertical = j.value("binning_vertical", 1);
		out.binning_selector = j.value("binning_selector", "Sensor");
		out.scaling_horizontal = j.value("scaling_horizontal", 1.0);
		out.reverse_x = j.value("reverse_x", false);
		out.reverse_y = j.value("reverse_y", false);
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

	const bool use_binning = s.binning_horizontal > 1 || s.binning_vertical > 1;
	const bool use_scaling = s.scaling_horizontal > 0.0
		&& s.scaling_horizontal < 1.0 - 1e-6;

	// Binning and scaling are mutually exclusive on ace 2.
	if (use_binning) {
		apply_binning_selector(cam, s.binning_selector);
		cam.BinningHorizontal.SetValue(s.binning_horizontal);
		cam.BinningVertical.SetValue(s.binning_vertical);
		cam.ScalingHorizontal.SetValue(1.0);
	} else if (use_scaling) {
		cam.BinningHorizontal.SetValue(1);
		cam.BinningVertical.SetValue(1);
		cam.ScalingHorizontal.SetValue(s.scaling_horizontal);
	} else {
		cam.BinningHorizontal.SetValue(1);
		cam.BinningVertical.SetValue(1);
		cam.ScalingHorizontal.SetValue(1.0);
	}

	cam.Width.SetValue(s.width);
	cam.Height.SetValue(s.height);
	cam.OffsetX.SetValue(s.offset_x);
	cam.OffsetY.SetValue(s.offset_y);

	cam.ReverseX.SetValue(s.reverse_x);
	cam.ReverseY.SetValue(s.reverse_y);

	cam.BlackLevel.SetValue(s.black_level);
	cam.Gamma.SetValue(s.gamma);

	apply_exposure_time_mode(cam, s.exposure_time_mode);

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

	apply_throughput_limit(cam, s);

	std::ostringstream oss;
	oss << "Applied " << s.pixel_format << " " << s.width << "x" << s.height
		<< " offset=(" << s.offset_x << "," << s.offset_y << ")"
		<< " binning=" << s.binning_horizontal << "x" << s.binning_vertical
		<< "(" << s.binning_selector << ")"
		<< " scale=" << s.scaling_horizontal
		<< " exposure_mode=" << s.exposure_time_mode
		<< " exposure=" << s.exposure_time_us << "us"
		<< " gain=" << s.gain_db << "dB"
		<< " black=" << s.black_level << " gamma=" << s.gamma
		<< " fps=" << s.frame_rate_fps;
	log_info("camera", oss.str());
}
