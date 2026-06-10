#pragma once

#include <nlohmann/json.hpp>
#include <pylon/BaslerUniversalInstantCamera.h>
#include <string>

#include "camera_settings.h"
#include "image_metrics.h"

// Shared capture helpers for test_param_sweep (parameter + resolution modes).

struct CaptureOptions {
	int frames = 50;
	int save_images = 3;
};

struct CaptureResult {
	ImageMetrics metrics;
	double achieved_fps = 0.0;
	int frames = 0;
};

// Apply one camera_config.json field override (param sweep).
bool apply_camera_override(CameraSettings& settings, const std::string& key,
	const nlohmann::json& value);

// Apply any known camera_config fields from a preset object (width, binning, …).
bool apply_sweep_preset(CameraSettings& settings, const nlohmann::json& preset);

// Back-compat alias used by resolution sweeps.
bool apply_resolution_preset(CameraSettings& settings,
	const nlohmann::json& preset);

// Filesystem-safe token for image filenames.
std::string sanitize_sweep_label(const std::string& raw);

// Grab frames at current camera settings; save up to save_images PNGs.
CaptureResult capture_frames(Pylon::CBaslerUniversalInstantCamera& camera,
	const CaptureOptions& opts, const std::string& session_dir,
	const std::string& file_prefix);
