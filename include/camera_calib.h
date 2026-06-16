#pragma once

#include <opencv2/opencv.hpp>
#include <optional>
#include <string>

// Distortion model stored in calib.npz (model_id array).
enum class CalibModel : int {
	Standard = 0,  // cv::calibrateCamera + rational coeffs
	Fisheye = 1,   // cv::fisheye::calibrate — better for 4 mm ~79° lens
};

// Intrinsics + precomputed undistort maps loaded from src/calibration.py calib.npz.
struct CameraCalib {
	CalibModel model = CalibModel::Standard;
	cv::Mat camera_matrix;   // 3×3 CV_64F
	cv::Mat dist_coeffs;       // standard: 1×N; fisheye: 4×1
	cv::Mat map1;              // CV_16SC2 remap table
	cv::Mat map2;              // CV_16UC1 remap table
	cv::Size image_size;
	double rms = 0.0;
	double undistort_alpha = 0.0;

	bool enabled() const { return !map1.empty(); }
};

// CLI > PYLON_CAMERA_CALIB env > calib.npz beside executable > ./calib.npz.
std::string resolve_calib_path(const char* argv0, const std::string& cli_path);

std::optional<CameraCalib> load_camera_calib(const std::string& path,
	cv::Size frame_size);
