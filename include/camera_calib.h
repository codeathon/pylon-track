#pragma once

#include <opencv2/opencv.hpp>
#include <optional>
#include <string>

// Intrinsics + precomputed undistort maps loaded from calibration.py calib.npz.
struct CameraCalib {
	cv::Mat camera_matrix;   // 3×3 CV_64F
	cv::Mat dist_coeffs;       // 1×N CV_64F (rational model → 8 coeffs)
	cv::Mat map1;              // CV_16SC2 remap table
	cv::Mat map2;              // CV_16UC1 remap table
	cv::Size image_size;
	double rms = 0.0;

	bool enabled() const { return !map1.empty(); }
};

// CLI > PYLON_CAMERA_CALIB env > calib.npz beside executable > ./calib.npz.
// Returns empty string when no file is found (undistort stays off).
std::string resolve_calib_path(const char* argv0, const std::string& cli_path);

// Load K/dist from npz and build remap tables for the live camera frame size.
// Fails when npz img_size does not match frame_size (recalibrate at this AOI).
std::optional<CameraCalib> load_camera_calib(const std::string& path,
	cv::Size frame_size);
