// Camera config sweep — one executable, three spec formats:
//
//   "parameter" + "values"     — one setting at a time
//   "presets"                  — AOI width×height (+ optional offsets)
//   "presets" + preset_type    — binning or compound camera presets
//
// Usage:
//   test_param_sweep --sweep tests/sweep_configs/exposure_sweep.json
//   test_param_sweep --sweep tests/sweep_configs/resolution_sweep.json [--gsd 1.035]

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

namespace fs = std::filesystem;
using nlohmann::json;

namespace {

enum class SweepMode { Parameter, Resolution, Binning, CameraPreset };

struct ParamSweepSpec {
	std::string parameter;
	std::vector<json> values;
	int frames_per_step = 50;
	int save_images = 3;
};

struct PresetSweepSpec {
	std::string preset_type = "resolution";
	double gsd_mm_px = GSD_MM_PX;
	int frames_per_step = 50;
	int save_images = 2;
	std::vector<json> presets;
};

bool load_spec_json(const std::string& path, json& out) {
	std::ifstream file(path);
	if (!file.is_open()) {
		log_error("sweep", "Cannot open sweep spec: " + path);
		return false;
	}
	try {
		file >> out;
	} catch (const std::exception& e) {
		log_error("sweep", std::string("JSON parse error: ") + e.what());
		return false;
	}
	return true;
}

SweepMode detect_mode(const json& j) {
	if (j.contains("presets")) {
		const std::string kind = j.value("preset_type", "resolution");
		if (kind == "binning") {
			return SweepMode::Binning;
		}
		if (kind == "camera") {
			return SweepMode::CameraPreset;
		}
		return SweepMode::Resolution;
	}
	if (j.contains("parameter")) {
		return SweepMode::Parameter;
	}
	log_error("sweep",
		"Spec needs 'parameter'+'values' or 'presets' — see tests/sweep_configs/");
	return SweepMode::Parameter;
}

bool parse_param_spec(const json& j, ParamSweepSpec& out) {
	try {
		out.parameter = j.at("parameter").get<std::string>();
		out.values = j.at("values").get<std::vector<json>>();
		out.frames_per_step = j.value("frames_per_value", 50);
		out.save_images = j.value("save_images", 3);
		return true;
	} catch (const std::exception& e) {
		log_error("sweep", std::string("Invalid parameter spec: ") + e.what());
		return false;
	}
}

bool parse_preset_spec(const json& j, PresetSweepSpec& out) {
	try {
		out.preset_type = j.value("preset_type", "resolution");
		out.gsd_mm_px = j.value("gsd_mm_px", GSD_MM_PX);
		out.frames_per_step = j.value("frames_per_preset", 50);
		out.save_images = j.value("save_images", 2);
		out.presets = j.at("presets").get<std::vector<json>>();
		if (out.presets.empty()) {
			log_error("sweep", "Preset spec has empty presets[]");
			return false;
		}
		return true;
	} catch (const std::exception& e) {
		log_error("sweep", std::string("Invalid preset spec: ") + e.what());
		return false;
	}
}

std::string value_label(const json& v) {
	if (v.is_boolean()) {
		return v.get<bool>() ? "true" : "false";
	}
	std::string s = v.is_string() ? v.get<std::string>() : v.dump();
	for (char& c : s) {
		if (c == '.') {
			c = 'p';
		}
	}
	return s;
}

std::string preset_label(const json& preset) {
	if (preset.contains("label")) {
		return sanitize_sweep_label(preset.at("label").get<std::string>());
	}
	if (preset.contains("width") && preset.contains("height")) {
		const int w = preset.at("width").get<int>();
		const int h = preset.at("height").get<int>();
		return "w" + std::to_string(w) + "h" + std::to_string(h);
	}
	return "preset";
}

bool apply_and_capture(Pylon::CBaslerUniversalInstantCamera& camera,
	CameraSettings& settings, const CaptureOptions& opts,
	const std::string& session_dir, const std::string& file_prefix,
	CaptureResult& out)
{
	try {
		configure_camera(camera, settings);
	} catch (const std::exception& e) {
		log_warn("sweep", std::string("Skipping: ") + e.what());
		return false;
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	out = capture_frames(camera, opts, session_dir, file_prefix);
	return true;
}

void write_metric_cells(const CaptureResult& result,
	std::vector<std::string>& cells)
{
	cells.push_back(std::to_string(result.frames));
	cells.push_back(csv_num(result.achieved_fps, 1));
	cells.push_back(csv_num(result.metrics.mean_gray, 2));
	cells.push_back(csv_num(result.metrics.stddev, 2));
	cells.push_back(csv_num(result.metrics.clipped_low_pct, 3));
	cells.push_back(csv_num(result.metrics.clipped_high_pct, 3));
	cells.push_back(csv_num(result.metrics.laplacian_var, 2));
}

int run_parameter_sweep(Pylon::CBaslerUniversalInstantCamera& camera,
	const CameraSettings& base, const ParamSweepSpec& spec,
	const std::string& output_base)
{
	const std::string session_dir =
		make_session_dir(output_base, "param_sweep", spec.parameter);
	CsvWriter csv((fs::path(session_dir) / "sweep.csv").string(), {
		spec.parameter, "frames", "achieved_fps", "mean_gray", "stddev",
		"clipped_low_pct", "clipped_high_pct", "laplacian_var",
	});
	const CaptureOptions opts{spec.frames_per_step, spec.save_images};

	for (const json& value : spec.values) {
		CameraSettings settings = base;
		if (!apply_camera_override(settings, spec.parameter, value)) {
			log_error("sweep", "Unknown parameter: " + spec.parameter);
			return 1;
		}
		log_info("sweep", spec.parameter + " = " + value.dump());
		CaptureResult result;
		const std::string prefix = spec.parameter + "_" + value_label(value);
		if (!apply_and_capture(camera, settings, opts, session_dir, prefix, result)) {
			continue;
		}
		csv.write_row({
			value_label(value),
			std::to_string(result.frames),
			csv_num(result.achieved_fps, 1),
			csv_num(result.metrics.mean_gray, 2),
			csv_num(result.metrics.stddev, 2),
			csv_num(result.metrics.clipped_low_pct, 3),
			csv_num(result.metrics.clipped_high_pct, 3),
			csv_num(result.metrics.laplacian_var, 2),
		});
	}
	log_info("sweep", "Parameter sweep done -> " + session_dir);
	return 0;
}

int run_resolution_sweep(Pylon::CBaslerUniversalInstantCamera& camera,
	const CameraSettings& base, const PresetSweepSpec& spec,
	const std::string& output_base)
{
	const std::string session_dir =
		make_session_dir(output_base, "param_sweep", "resolution");
	CsvWriter csv((fs::path(session_dir) / "resolution.csv").string(), {
		"label", "width", "height", "offset_x", "offset_y", "total_px",
		"fov_width_mm", "fov_height_mm", "megapixels",
		"frames", "achieved_fps", "mean_gray", "stddev",
		"clipped_low_pct", "clipped_high_pct", "laplacian_var",
	});
	const CaptureOptions opts{spec.frames_per_step, spec.save_images};

	for (const json& preset : spec.presets) {
		CameraSettings settings = base;
		if (!apply_sweep_preset(settings, preset)) {
			log_warn("sweep", "Skipping preset with no recognized fields");
			continue;
		}
		if (!preset.contains("width") || !preset.contains("height")) {
			log_warn("sweep", "Skipping resolution preset missing width/height");
			continue;
		}
		log_info("sweep", "preset " + preset_label(preset) + " "
			+ std::to_string(settings.width) + "x"
			+ std::to_string(settings.height));
		CaptureResult result;
		const std::string prefix = "resolution_" + preset_label(preset);
		if (!apply_and_capture(camera, settings, opts, session_dir, prefix, result)) {
			continue;
		}
		const int w = settings.width;
		const int h = settings.height;
		std::vector<std::string> row = {
			preset_label(preset),
			std::to_string(w), std::to_string(h),
			std::to_string(settings.offset_x), std::to_string(settings.offset_y),
			std::to_string(w * h),
			csv_num(w * spec.gsd_mm_px, 1),
			csv_num(h * spec.gsd_mm_px, 1),
			csv_num(static_cast<double>(w * h) / 1e6, 3),
		};
		write_metric_cells(result, row);
		csv.write_row(row);
	}
	log_info("sweep", "Resolution sweep done -> " + session_dir);
	return 0;
}

int run_binning_sweep(Pylon::CBaslerUniversalInstantCamera& camera,
	const CameraSettings& base, const PresetSweepSpec& spec,
	const std::string& output_base)
{
	const std::string session_dir =
		make_session_dir(output_base, "param_sweep", "binning");
	CsvWriter csv((fs::path(session_dir) / "binning.csv").string(), {
		"label", "binning_selector", "binning_horizontal", "binning_vertical",
		"output_width", "output_height",
		"frames", "achieved_fps", "mean_gray", "stddev",
		"clipped_low_pct", "clipped_high_pct", "laplacian_var",
	});
	const CaptureOptions opts{spec.frames_per_step, spec.save_images};

	for (const json& preset : spec.presets) {
		CameraSettings settings = base;
		settings.scaling_horizontal = 1.0;
		if (!apply_sweep_preset(settings, preset)) {
			log_warn("sweep", "Skipping binning preset with no fields");
			continue;
		}
		log_info("sweep", "binning " + preset_label(preset));
		CaptureResult result;
		const std::string prefix = "binning_" + preset_label(preset);
		if (!apply_and_capture(camera, settings, opts, session_dir, prefix, result)) {
			continue;
		}
		std::vector<std::string> row = {
			preset_label(preset),
			settings.binning_selector,
			std::to_string(settings.binning_horizontal),
			std::to_string(settings.binning_vertical),
			std::to_string(settings.width),
			std::to_string(settings.height),
		};
		write_metric_cells(result, row);
		csv.write_row(row);
	}
	log_info("sweep", "Binning sweep done -> " + session_dir);
	return 0;
}

int run_camera_preset_sweep(Pylon::CBaslerUniversalInstantCamera& camera,
	const CameraSettings& base, const PresetSweepSpec& spec,
	const std::string& output_base)
{
	const std::string session_dir =
		make_session_dir(output_base, "param_sweep", "camera_preset");
	CsvWriter csv((fs::path(session_dir) / "camera_preset.csv").string(), {
		"label", "exposure_time_mode", "exposure_time_us",
		"scaling_horizontal", "device_link_throughput_mbps",
		"frames", "achieved_fps", "mean_gray", "stddev",
		"clipped_low_pct", "clipped_high_pct", "laplacian_var",
	});
	const CaptureOptions opts{spec.frames_per_step, spec.save_images};

	for (const json& preset : spec.presets) {
		CameraSettings settings = base;
		if (!apply_sweep_preset(settings, preset)) {
			log_warn("sweep", "Skipping camera preset with no fields");
			continue;
		}
		log_info("sweep", "camera preset " + preset_label(preset));
		CaptureResult result;
		const std::string prefix = "camera_" + preset_label(preset);
		if (!apply_and_capture(camera, settings, opts, session_dir, prefix, result)) {
			continue;
		}
		std::vector<std::string> row = {
			preset_label(preset),
			settings.exposure_time_mode,
			csv_num(settings.exposure_time_us, 1),
			csv_num(settings.scaling_horizontal, 3),
			csv_num(settings.device_link_throughput_mbps, 1),
		};
		write_metric_cells(result, row);
		csv.write_row(row);
	}
	log_info("sweep", "Camera preset sweep done -> " + session_dir);
	return 0;
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
		std::cerr << "Usage: test_param_sweep --sweep <spec.json> "
			"[--gsd <mm/px>] [--camera-config <path>] [--output <dir>]\n";
		return 1;
	}

	json root;
	if (!load_spec_json(sweep_path, root)) {
		return 1;
	}
	if (!root.contains("presets") && !root.contains("parameter")) {
		log_error("sweep",
			"Spec needs either 'parameter'+'values' or 'presets'");
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

		const SweepMode mode = detect_mode(root);
		if (mode == SweepMode::Parameter) {
			ParamSweepSpec spec;
			if (!parse_param_spec(root, spec)) {
				exit_code = 1;
			} else {
				exit_code = run_parameter_sweep(camera, base, spec, output_base);
			}
		} else {
			PresetSweepSpec spec;
			if (!parse_preset_spec(root, spec)) {
				exit_code = 1;
			} else {
				if (gsd_cli > 0.0) {
					spec.gsd_mm_px = gsd_cli;
				}
				if (mode == SweepMode::Resolution) {
					exit_code = run_resolution_sweep(camera, base, spec, output_base);
				} else if (mode == SweepMode::Binning) {
					exit_code = run_binning_sweep(camera, base, spec, output_base);
				} else {
					exit_code = run_camera_preset_sweep(camera, base, spec, output_base);
				}
			}
		}
	} catch (const Pylon::GenericException& e) {
		log_error("sweep", std::string("Pylon error: ") + e.GetDescription());
		exit_code = 1;
	}
	Pylon::PylonTerminate();
	return exit_code;
}
