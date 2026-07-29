#pragma once

#include <opencv2/core.hpp>
#include <vector>

struct CharucoDetectResult {
	std::vector<cv::Point2f> charuco_corners;
	std::vector<int> charuco_ids;
	int corner_count = 0;
};

// ChArUco board constants matching the legacy calibration script.
class CharucoBoardSpec {
public:
	static cv::Size board_size();
	static float square_length_m();
	static float marker_length_m();

	// Detect charuco corners in a mono8 frame.
	static CharucoDetectResult detect(const cv::Mat& gray);

	// Object/image point pairs for one view (empty if detection failed).
	static bool match_view(const CharucoDetectResult& det,
		std::vector<cv::Point3f>& obj_points,
		std::vector<cv::Point2f>& img_points);
};
