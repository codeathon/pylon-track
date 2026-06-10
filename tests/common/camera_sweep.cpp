#include "camera_sweep.h"

#include <chrono>
#include <cstdio>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include "test_session.h"

namespace fs = std::filesystem;
using nlohmann::json;

bool apply_camera_override(CameraSettings& s, const std::string& key, const json& v) {
	if (key == "exposure_time_us") { s.exposure_time_us = v.get<double>(); return true; }
	if (key == "exposure_time_mode") { s.exposure_time_mode = v.get<std::string>(); return true; }
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
	if (key == "device_link_throughput_mbps") {
		s.device_link_throughput_mbps = v.get<double>(); return true;
	}
	if (key == "black_level") { s.black_level = v.get<int>(); return true; }
	if (key == "gamma") { s.gamma = v.get<double>(); return true; }
	if (key == "binning_horizontal") { s.binning_horizontal = v.get<int>(); return true; }
	if (key == "binning_vertical") { s.binning_vertical = v.get<int>(); return true; }
	if (key == "binning_selector") { s.binning_selector = v.get<std::string>(); return true; }
	if (key == "scaling_horizontal") { s.scaling_horizontal = v.get<double>(); return true; }
	if (key == "reverse_x") { s.reverse_x = v.get<bool>(); return true; }
	if (key == "reverse_y") { s.reverse_y = v.get<bool>(); return true; }
	return false;
}

bool apply_resolution_preset(CameraSettings& s, const json& preset) {
	return apply_sweep_preset(s, preset);
}

bool apply_sweep_preset(CameraSettings& s, const json& preset) {
	bool applied = false;
	for (auto it = preset.begin(); it != preset.end(); ++it) {
		const std::string& key = it.key();
		if (key == "label") {
			continue;
		}
		if (apply_camera_override(s, key, it.value())) {
			applied = true;
		}
	}
	if (!preset.contains("width") || !preset.contains("height")) {
		return applied;
	}
	return true;
}

std::string sanitize_sweep_label(const std::string& raw) {
	std::string out;
	out.reserve(raw.size());
	for (char c : raw) {
		if ((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z')
			|| (c >= '0' && c <= '9') || c == '_' || c == '-') {
			out.push_back(c);
		} else if (c == '.' || c == ' ') {
			out.push_back('_');
		}
	}
	return out.empty() ? "preset" : out;
}

CaptureResult capture_frames(Pylon::CBaslerUniversalInstantCamera& camera,
	const CaptureOptions& opts, const std::string& session_dir,
	const std::string& file_prefix)
{
	using SteadyClock = std::chrono::steady_clock;
	std::vector<ImageMetrics> per_frame;
	CaptureResult result;

	camera.StartGrabbing(opts.frames, Pylon::GrabStrategy_OneByOne);
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

		if (result.frames < opts.save_images) {
			char name[192];
			std::snprintf(name, sizeof(name), "%s_%s_f%03d.png",
				timestamp_label().c_str(), file_prefix.c_str(), result.frames);
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
