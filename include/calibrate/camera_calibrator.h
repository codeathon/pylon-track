#pragma once

#include "calibrate/setup_options.h"
#include <opencv2/core.hpp>

// ChArUco capture + lens calibration → calib.npz.
class CameraCalibrator {
public:
	explicit CameraCalibrator(const SetupOptions& opts);

	bool run(const std::string& calib_output_path, cv::Size expected_size);

private:
	SetupOptions opts_;

	bool capture_frames(const std::string& camera_config_path, cv::Size& img_size);
	bool calibrate_from_frames(const std::string& calib_output_path, cv::Size img_size);
};
