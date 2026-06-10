// Suite 1 — camera_config parameter sweep.
//
// Holds every camera setting at its camera_config.json baseline and steps ONE
// parameter through a list of values from a sweep spec JSON. For each value it
// captures frames, saves labeled sample images, and records image-quality
// metrics so the best candidate value can be picked from one CSV.
//
// Usage:
//   test_param_sweep --sweep tests/sweep_configs/exposure_sweep.json \
//       [--camera-config path] [--output tests/output] [--verbose]

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "camera_config.h"
#include "camera_settings.h"
#include "logger.h"
#include "image_metrics.h"
#include "test_session.h"

namespace fs = std::filesystem;
using nlohmann::json;

namespace {

struct SweepSpec {
	std::string parameter;
	std::vector<json> values;
	int frames_per_value = 50;
	int save_images = 3;
};

bool load_sweep_spec(const std::string& path, SweepSpec& out) {
	std::ifstream file(path);
	if (!file.is_open()) {
		log_error("sweep", "Cannot open sweep spec: " + path);
		return false;
	}
	try {
		json j;
		file >> j;
		out.parameter = j.at("parameter").get<std::string>();
		out.values = j.at("values").get<std::vector<json>>();
		out.frames_per_value = j.value("frames_per_value", 50);
		out.save_images = j.value("save_images", 3);
	} catch (const std::exception& e) {
		log_error("sweep", std::string("Invalid sweep spec: ") + e.what());
		return false;
	}
	return true;
}

// Maps a sweep spec key onto the CameraSettings field. Mirrors the fields in
// camera_settings.h — extend both together when new settings are added.
bool apply_override(CameraSettings& s, const std::string& key, const json& v) {
	if (key == "exposure_time_us") { s.exposure_time_us = v.get<double>(); return true; }
	if (key == "gain_db") { s.gain_db = v.get<double>(); return true; }
	if (key == "frame_rate_fps") { s.frame_rate_fps = v.get<double>(); return true; }
	if (key == "width") { s.width = v.get<int>(); return true; }
	if (key == "height") { s.height = v.get<int>(); return true; }
	if (key == "offset_x") { s.offset_x = v.get<int>(); return true; }
	if (key == "offset_y") { s.offset_y = v.get<int>(); return true; }
	if (key == "exposure_auto") { s.exposure_auto = v.get<bool>(); return true; }
	if (key == "gain_auto") { s.gain_auto = v.get<bool>(); return true; }
	if (key == "frame_rate_enable") { s.frame_rate_enable = v.get<bool>(); return true; }
	if (key == "pixel_format") { s.pixel_format = v.get<std::string>(); return true; }
	if (key == "trigger_mode") { s.trigger_mode = v.get<std::string>(); return true; }
	if (key == "device_link_throughput_limit") {
		s.device_link_throughput_limit = v.get<std::string>(); return true;
	}
	return false;
}

// "2000.5" -> "2000p5" — keeps filenames shell- and Windows-safe.
std::string value_label(const json& v) {
	std::string s = v.is_string() ? v.get<std::string>() : v.dump();
	for (char& c : s) {
		if (c == '.') c = 'p';
	}
	return s;
}

struct ValueResult {
	ImageMetrics metrics;
	double achieved_fps = 0.0;
	int frames = 0;
};

// Grabs spec.frames_per_value frames at the current settings, computing
// metrics on each and saving the first spec.save_images as labeled PNGs.
ValueResult capture_for_value(Pylon::CBaslerUniversalInstantCamera& camera,
	const SweepSpec& spec, const json& value, const std::string& session_dir)
{
	using SteadyClock = std::chrono::steady_clock;
	std::vector<ImageMetrics> per_frame;
	ValueResult result;

	camera.StartGrabbing(spec.frames_per_value, Pylon::GrabStrategy_OneByOne);
	SteadyClock::time_point first_ts{}, last_ts{};

	Pylon::CGrabResultPtr grab;
	while (camera.IsGrabbing()) {
		camera.RetrieveResult(5000, grab, Pylon::TimeoutHandling_ThrowException);
		if (!grab->GrabSucceeded()) {
			continue;
		}
		const auto now = SteadyClock::now();
		if (result.frames == 0) {
			first_ts = now;
		}
		last_ts = now;

		const cv::Mat frame(grab->GetHeight(), grab->GetWidth(),
			CV_8UC1, grab->GetBuffer());
		per_frame.push_back(compute_image_metrics(frame));

		if (result.frames < spec.save_images) {
			char name[160];
			std::snprintf(name, sizeof(name), "%s_%s_%s_f%03d.png",
				timestamp_label().c_str(), spec.parameter.c_str(),
				value_label(value).c_str(), result.frames);
			cv::imwrite((fs::path(session_dir) / name).string(), frame);
		}
		++result.frames;
	}
	camera.StopGrabbing();

	result.metrics = average_metrics(per_frame);
	if (result.frames > 1) {
		const double span_s = std::chrono::duration<double>(last_ts - first_ts).count();
		result.achieved_fps = span_s > 0.0 ? (result.frames - 1) / span_s : 0.0;
	}
	return result;
}

void write_result_row(CsvWriter& csv, const json& value, const ValueResult& r) {
	csv.write_row({
		value.is_string() ? value.get<std::string>() : value.dump(),
		std::to_string(r.frames),
		csv_num(r.achieved_fps, 1),
		csv_num(r.metrics.mean_gray, 2),
		csv_num(r.metrics.stddev, 2),
		csv_num(r.metrics.clipped_low_pct, 3),
		csv_num(r.metrics.clipped_high_pct, 3),
		csv_num(r.metrics.laplacian_var, 2),
	});
}

} // namespace

int main(int argc, char** argv) {
	std::string sweep_path, config_path_cli, output_base = "tests/output";
	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "--sweep") == 0 && i + 1 < argc) {
			sweep_path = argv[++i];
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
	if (sweep_path.empty()) {
		std::cerr << "Usage: test_param_sweep --sweep <spec.json> "
			"[--camera-config <path>] [--output <dir>]\n";
		return 1;
	}

	SweepSpec spec;
	if (!load_sweep_spec(sweep_path, spec)) {
		return 1;
	}

	CameraSettings base;
	const std::string config_path = resolve_camera_config_path(argv[0], config_path_cli);
	if (!load_camera_config(config_path, base)) {
		return 1;
	}

	Pylon::PylonInitialize();
	int exit_code = 0;
	try {
		Pylon::CBaslerUniversalInstantCamera camera(
			Pylon::CTlFactory::GetInstance().CreateFirstDevice());
		log_info("sweep", "Camera: "
			+ std::string(camera.GetDeviceInfo().GetModelName()));

		const std::string session_dir =
			make_session_dir(output_base, "param_sweep", spec.parameter);
		CsvWriter csv((fs::path(session_dir) / "sweep.csv").string(), {
			spec.parameter, "frames", "achieved_fps", "mean_gray", "stddev",
			"clipped_low_pct", "clipped_high_pct", "laplacian_var",
		});

		for (const json& value : spec.values) {
			CameraSettings settings = base;
			if (!apply_override(settings, spec.parameter, value)) {
				log_error("sweep", "Unknown parameter: " + spec.parameter);
				exit_code = 1;
				break;
			}
			log_info("sweep", spec.parameter + " = " + value.dump());
			try {
				configure_camera(camera, settings);
			} catch (const std::exception& e) {
				// Skip unsupported values (e.g. enum strings the camera
				// rejects) so the rest of the sweep still completes.
				log_warn("sweep", std::string("Skipping value: ") + e.what());
				continue;
			}
			// Let auto-functions / sensor settle before measuring.
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			write_result_row(csv, value, capture_for_value(camera, spec, value, session_dir));
		}
		log_info("sweep", "Done -> " + session_dir);
	} catch (const Pylon::GenericException& e) {
		log_error("sweep", std::string("Pylon error: ") + e.GetDescription());
		exit_code = 1;
	}
	Pylon::PylonTerminate();
	return exit_code;
}
