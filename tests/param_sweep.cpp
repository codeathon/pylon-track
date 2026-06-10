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

std::string value_label(const json& v) {
	std::string s = v.is_string() ? v.get<std::string>() : v.dump();
	for (char& c : s) {
		if (c == '.') {
			c = 'p';
		}
	}
	return s;
}

void write_result_row(CsvWriter& csv, const std::string& parameter,
	const json& value, const CaptureResult& r)
{
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
		CsvWriter csv((std::filesystem::path(session_dir) / "sweep.csv").string(), {
			spec.parameter, "frames", "achieved_fps", "mean_gray", "stddev",
			"clipped_low_pct", "clipped_high_pct", "laplacian_var",
		});

		const CaptureOptions capture_opts{spec.frames_per_value, spec.save_images};

		for (const json& value : spec.values) {
			CameraSettings settings = base;
			if (!apply_camera_override(settings, spec.parameter, value)) {
				log_error("sweep", "Unknown parameter: " + spec.parameter);
				exit_code = 1;
				break;
			}
			log_info("sweep", spec.parameter + " = " + value.dump());
			try {
				configure_camera(camera, settings);
			} catch (const std::exception& e) {
				log_warn("sweep", std::string("Skipping value: ") + e.what());
				continue;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			const std::string prefix = spec.parameter + "_" + value_label(value);
			write_result_row(csv, spec.parameter, value,
				capture_frames(camera, capture_opts, session_dir, prefix));
		}
		log_info("sweep", "Done -> " + session_dir);
	} catch (const Pylon::GenericException& e) {
		log_error("sweep", std::string("Pylon error: ") + e.GetDescription());
		exit_code = 1;
	}
	Pylon::PylonTerminate();
	return exit_code;
}
