// One-time camera setup validation — apply fixed rig settings once, capture a
// verification frame, read back GenICam values, and write a setup report.
// Flat-field and vignetting correction still require pylon Viewer (manual_steps).
//
// Usage:
//   test_one_time_setup [--settings tests/one_time_settings.json]
//       [--camera-config path] [--output tests/output] [--verbose]

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

#include <nlohmann/json.hpp>

#include "camera_config.h"
#include "camera_settings.h"
#include "camera_sweep.h"
#include "logger.h"
#include "test_session.h"

namespace fs = std::filesystem;
using nlohmann::json;

namespace {

struct OneTimeSpec {
	std::string description;
	std::vector<std::string> manual_steps;
	CameraSettings camera;
	int verify_frames = 30;
	int save_images = 1;
	double min_mean_gray = 40.0;
	double max_mean_gray = 200.0;
	double max_clipped_pct = 2.0;
};

bool load_one_time_spec(const std::string& path, OneTimeSpec& out) {
	std::ifstream file(path);
	if (!file.is_open()) {
		log_error("one_time", "Cannot open settings: " + path);
		return false;
	}
	json root;
	try {
		file >> root;
	} catch (const std::exception& e) {
		log_error("one_time", std::string("JSON parse error: ") + e.what());
		return false;
	}

	out.description = root.value("description", "");
	out.manual_steps.clear();
	if (root.contains("manual_steps")) {
		out.manual_steps = root.at("manual_steps").get<std::vector<std::string>>();
	}

	// Write camera block to a temp file so we reuse the production loader.
	const fs::path tmp = fs::temp_directory_path() / "pylon_one_time_camera.json";
	{
		std::ofstream tmp_file(tmp);
		tmp_file << root.at("camera").dump(4);
	}
	if (!load_camera_config(tmp.string(), out.camera)) {
		return false;
	}

	const json verify = root.value("verification", json::object());
	out.verify_frames = verify.value("frames", 30);
	out.save_images = verify.value("save_images", 1);
	out.min_mean_gray = verify.value("min_mean_gray", 40.0);
	out.max_mean_gray = verify.value("max_mean_gray", 200.0);
	out.max_clipped_pct = verify.value("max_clipped_pct", 2.0);
	return true;
}

json camera_settings_to_json(const CameraSettings& s) {
	return {
		{"pixel_format", s.pixel_format},
		{"width", s.width},
		{"height", s.height},
		{"offset_x", s.offset_x},
		{"offset_y", s.offset_y},
		{"exposure_auto", s.exposure_auto},
		{"exposure_time_us", s.exposure_time_us},
		{"exposure_time_mode", s.exposure_time_mode},
		{"gain_auto", s.gain_auto},
		{"gain_db", s.gain_db},
		{"frame_rate_enable", s.frame_rate_enable},
		{"frame_rate_fps", s.frame_rate_fps},
		{"trigger_mode", s.trigger_mode},
		{"device_link_throughput_limit", s.device_link_throughput_limit},
		{"device_link_throughput_mbps", s.device_link_throughput_mbps},
		{"black_level", s.black_level},
		{"gamma", s.gamma},
		{"binning_horizontal", s.binning_horizontal},
		{"binning_vertical", s.binning_vertical},
		{"binning_selector", s.binning_selector},
		{"scaling_horizontal", s.scaling_horizontal},
		{"reverse_x", s.reverse_x},
		{"reverse_y", s.reverse_y},
	};
}

json read_camera_state(Pylon::CBaslerUniversalInstantCamera& camera) {
	json state;
	state["pixel_format"] = camera.PixelFormat.GetValue();
	state["width"] = camera.Width.GetValue();
	state["height"] = camera.Height.GetValue();
	state["offset_x"] = camera.OffsetX.GetValue();
	state["offset_y"] = camera.OffsetY.GetValue();
	state["binning_horizontal"] = camera.BinningHorizontal.GetValue();
	state["binning_vertical"] = camera.BinningVertical.GetValue();
	state["scaling_horizontal"] = camera.ScalingHorizontal.GetValue();
	state["reverse_x"] = camera.ReverseX.GetValue();
	state["reverse_y"] = camera.ReverseY.GetValue();
	state["black_level"] = camera.BlackLevel.GetValue();
	state["gamma"] = camera.Gamma.GetValue();
	state["exposure_time_us"] = camera.ExposureTime.GetValue();
	state["gain_db"] = camera.Gain.GetValue();
	state["frame_rate_enable"] = camera.AcquisitionFrameRateEnable.GetValue();
	state["frame_rate_fps"] = camera.AcquisitionFrameRate.GetValue();
	state["trigger_mode"] = camera.TriggerMode.GetValue();
	if (camera.ResultingFrameRate.IsReadable()) {
		state["resulting_fps"] = camera.ResultingFrameRate.GetValue();
	}
	return state;
}

bool verify_metrics(const CaptureResult& result, const OneTimeSpec& spec,
	std::vector<std::string>& warnings)
{
	bool ok = true;
	const double clip = result.metrics.clipped_low_pct + result.metrics.clipped_high_pct;
	if (result.metrics.mean_gray < spec.min_mean_gray
		|| result.metrics.mean_gray > spec.max_mean_gray) {
		warnings.push_back("mean_gray " + csv_num(result.metrics.mean_gray, 2)
			+ " outside [" + csv_num(spec.min_mean_gray, 1) + ", "
			+ csv_num(spec.max_mean_gray, 1) + "] — adjust exposure/gain or lighting");
		ok = false;
	}
	if (clip > spec.max_clipped_pct) {
		warnings.push_back("clipped pixels " + csv_num(clip, 3)
			+ "% exceed max " + csv_num(spec.max_clipped_pct, 1) + "%");
		ok = false;
	}
	if (result.achieved_fps < spec.camera.frame_rate_fps * 0.9
		&& spec.camera.frame_rate_enable) {
		warnings.push_back("achieved_fps " + csv_num(result.achieved_fps, 1)
			+ " below 90% of target " + csv_num(spec.camera.frame_rate_fps, 1));
		ok = false;
	}
	return ok;
}

} // namespace

int main(int argc, char** argv) {
	std::string settings_path = "tests/one_time_settings.json";
	std::string config_path_cli, output_base = "tests/output";

	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "--settings") == 0 && i + 1 < argc) {
			settings_path = argv[++i];
		} else if (std::strcmp(argv[i], "--camera-config") == 0 && i + 1 < argc) {
			config_path_cli = argv[++i];
		} else if (std::strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
			output_base = argv[++i];
		} else if (std::strcmp(argv[i], "--verbose") == 0) {
			Logger::instance().set_level(LogLevel::Debug);
		} else {
			std::cerr << "Unknown argument: " << argv[i] << '\n';
			return 1;
		}
	}

	OneTimeSpec spec;
	if (!load_one_time_spec(settings_path, spec)) {
		return 1;
	}

	// Optional baseline from camera_config.json — one_time camera block wins.
	if (!config_path_cli.empty()) {
		CameraSettings baseline;
		if (load_camera_config(config_path_cli, baseline)) {
			log_info("one_time", "Using --camera-config only for path reference; "
				"applied values come from --settings camera block");
		}
	}

	const std::string session_dir =
		make_session_dir(output_base, "one_time_setup", "rig");

	Pylon::PylonInitialize();
	int exit_code = 0;
	try {
		Pylon::CBaslerUniversalInstantCamera camera(
			Pylon::CTlFactory::GetInstance().CreateFirstDevice());
		const std::string model = camera.GetDeviceInfo().GetModelName();
		log_info("one_time", "Camera: " + std::string(model));

		configure_camera(camera, spec.camera);
		std::this_thread::sleep_for(std::chrono::milliseconds(500));

		CaptureOptions opts{spec.verify_frames, spec.save_images};
		const CaptureResult capture = capture_frames(camera, opts, session_dir,
			"one_time_verify");

		std::vector<std::string> warnings;
		const bool metrics_ok = verify_metrics(capture, spec, warnings);

		json report;
		report["timestamp"] = timestamp_label();
		report["camera_model"] = model;
		report["description"] = spec.description;
		report["manual_steps_remaining"] = spec.manual_steps;
		report["applied_settings"] = camera_settings_to_json(spec.camera);
		report["readback"] = read_camera_state(camera);
		report["verification"] = {
			{"frames", capture.frames},
			{"achieved_fps", capture.achieved_fps},
			{"mean_gray", capture.metrics.mean_gray},
			{"stddev", capture.metrics.stddev},
			{"clipped_low_pct", capture.metrics.clipped_low_pct},
			{"clipped_high_pct", capture.metrics.clipped_high_pct},
			{"laplacian_var", capture.metrics.laplacian_var},
			{"passed", metrics_ok},
		};
		report["warnings"] = warnings;

		const fs::path report_path = fs::path(session_dir) / "setup_report.json";
		std::ofstream out(report_path);
		out << report.dump(4);
		log_info("one_time", "Report -> " + report_path.string());

		if (!metrics_ok) {
			for (const std::string& w : warnings) {
				log_warn("one_time", w);
			}
			exit_code = 1;
		} else {
			log_info("one_time", "One-time setup verification passed");
		}
	} catch (const std::exception& e) {
		log_error("one_time", std::string("Error: ") + e.what());
		exit_code = 1;
	} catch (const Pylon::GenericException& e) {
		log_error("one_time", std::string("Pylon error: ") + e.GetDescription());
		exit_code = 1;
	}
	Pylon::PylonTerminate();
	return exit_code;
}
