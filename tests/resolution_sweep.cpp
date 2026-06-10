// Suite 4 — AOI / resolution preset sweep.
//
// Steps through width×height (and optional offset) presets from a sweep spec.
// Cropping changes field-of-view in mm and achievable fps, not mm/px per pixel
// (GSD is optics + mount height). Use this to pick the AOI that balances
// arena coverage vs frame rate while keeping enough pixels on target.
//
// Usage:
//   test_resolution_sweep --sweep tests/sweep_configs/resolution_sweep.json
//       [--camera-config path] [--gsd 1.035] [--output tests/output] [--verbose]

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
#include "ferret_tracker.h"
#include "logger.h"
#include "test_session.h"

using nlohmann::json;

namespace {

struct ResolutionSweepSpec {
	double gsd_mm_px = GSD_MM_PX;
	int frames_per_preset = 50;
	int save_images = 2;
	std::vector<json> presets;
};

bool load_resolution_spec(const std::string& path, ResolutionSweepSpec& out) {
	std::ifstream file(path);
	if (!file.is_open()) {
		log_error("resolution", "Cannot open sweep spec: " + path);
		return false;
	}
	try {
		json j;
		file >> j;
		out.gsd_mm_px = j.value("gsd_mm_px", GSD_MM_PX);
		out.frames_per_preset = j.value("frames_per_preset", 50);
		out.save_images = j.value("save_images", 2);
		out.presets = j.at("presets").get<std::vector<json>>();
	} catch (const std::exception& e) {
		log_error("resolution", std::string("Invalid sweep spec: ") + e.what());
		return false;
	}
	return !out.presets.empty();
}

std::string preset_label(const json& preset) {
	if (preset.contains("label")) {
		return sanitize_sweep_label(preset.at("label").get<std::string>());
	}
	const int w = preset.at("width").get<int>();
	const int h = preset.at("height").get<int>();
	return "w" + std::to_string(w) + "h" + std::to_string(h);
}

std::string file_prefix_for_preset(const json& preset) {
	return "resolution_" + preset_label(preset);
}

void write_resolution_row(CsvWriter& csv, const json& preset, double gsd_mm_px,
	const CaptureResult& r)
{
	const int w = preset.at("width").get<int>();
	const int h = preset.at("height").get<int>();
	const int ox = preset.value("offset_x", 0);
	const int oy = preset.value("offset_y", 0);
	const int total_px = w * h;

	csv.write_row({
		preset_label(preset),
		std::to_string(w),
		std::to_string(h),
		std::to_string(ox),
		std::to_string(oy),
		std::to_string(total_px),
		csv_num(w * gsd_mm_px, 1),
		csv_num(h * gsd_mm_px, 1),
		csv_num(static_cast<double>(total_px) / 1e6, 3),
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
	double gsd_cli = -1.0;

	for (int i = 1; i < argc; ++i) {
		if (std::strcmp(argv[i], "--sweep") == 0 && i + 1 < argc) {
			sweep_path = argv[++i];
		} else if (std::strcmp(argv[i], "--camera-config") == 0 && i + 1 < argc) {
			config_path_cli = argv[++i];
		} else if (std::strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
			output_base = argv[++i];
		} else if (std::strcmp(argv[i], "--gsd") == 0 && i + 1 < argc) {
			gsd_cli = std::stod(argv[++i]);
		} else if (std::strcmp(argv[i], "--verbose") == 0) {
			Logger::instance().set_level(LogLevel::Debug);
		} else {
			std::cerr << "Unknown argument: " << argv[i] << '\n';
			return 1;
		}
	}
	if (sweep_path.empty()) {
		std::cerr << "Usage: test_resolution_sweep --sweep <spec.json> "
			"[--gsd <mm/px>] [--camera-config <path>] [--output <dir>]\n";
		return 1;
	}

	ResolutionSweepSpec spec;
	if (!load_resolution_spec(sweep_path, spec)) {
		return 1;
	}
	if (gsd_cli > 0.0) {
		spec.gsd_mm_px = gsd_cli;
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
		log_info("resolution", "Camera: "
			+ std::string(camera.GetDeviceInfo().GetModelName()));

		const std::string session_dir =
			make_session_dir(output_base, "resolution_sweep", "");
		CsvWriter csv((std::filesystem::path(session_dir) / "resolution.csv").string(), {
			"label", "width", "height", "offset_x", "offset_y", "total_px",
			"fov_width_mm", "fov_height_mm", "megapixels",
			"frames", "achieved_fps", "mean_gray", "stddev",
			"clipped_low_pct", "clipped_high_pct", "laplacian_var",
		});

		const CaptureOptions capture_opts{
			spec.frames_per_preset, spec.save_images,
		};

		for (const json& preset : spec.presets) {
			CameraSettings settings = base;
			if (!apply_resolution_preset(settings, preset)) {
				log_warn("resolution", "Skipping preset missing width/height");
				continue;
			}
			log_info("resolution", preset_label(preset) + " "
				+ std::to_string(settings.width) + "x"
				+ std::to_string(settings.height)
				+ " offset=(" + std::to_string(settings.offset_x) + ","
				+ std::to_string(settings.offset_y) + ")");
			try {
				configure_camera(camera, settings);
			} catch (const std::exception& e) {
				log_warn("resolution", std::string("Skipping preset: ") + e.what());
				continue;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			const CaptureResult result = capture_frames(
				camera, capture_opts, session_dir, file_prefix_for_preset(preset));
			write_resolution_row(csv, preset, spec.gsd_mm_px, result);
		}
		log_info("resolution", "Done -> " + session_dir);
	} catch (const Pylon::GenericException& e) {
		log_error("resolution", std::string("Pylon error: ") + e.GetDescription());
		exit_code = 1;
	}
	Pylon::PylonTerminate();
	return exit_code;
}
