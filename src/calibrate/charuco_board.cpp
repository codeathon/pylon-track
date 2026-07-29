#include "calibrate/charuco_board.h"

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>

namespace {

constexpr int kSquaresX = 10;
constexpr int kSquaresY = 7;
constexpr float kSquareLen = 0.035f;
constexpr float kMarkerLen = 0.026f;
constexpr int kMinCorners = 8;

cv::Ptr<cv::aruco::CharucoBoard> make_board() {
	const cv::Ptr<cv::aruco::Dictionary> dict =
		cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
	return cv::aruco::CharucoBoard::create(
		cv::Size(kSquaresX, kSquaresY), kSquareLen, kMarkerLen, dict);
}

cv::Ptr<cv::aruco::DetectorParameters> make_detector_params() {
	cv::Ptr<cv::aruco::DetectorParameters> params =
		cv::aruco::DetectorParameters::create();
	params->adaptiveThreshWinSizeMin = 3;
	params->adaptiveThreshWinSizeMax = 33;
	params->adaptiveThreshWinSizeStep = 4;
	params->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
	return params;
}

} // namespace

cv::Size CharucoBoardSpec::board_size() {
	return cv::Size(kSquaresX, kSquaresY);
}

float CharucoBoardSpec::square_length_m() {
	return kSquareLen;
}

float CharucoBoardSpec::marker_length_m() {
	return kMarkerLen;
}

CharucoDetectResult CharucoBoardSpec::detect(const cv::Mat& gray) {
	CharucoDetectResult out;
	if (gray.empty()) {
		return out;
	}
	std::vector<std::vector<cv::Point2f>> marker_corners;
	std::vector<int> marker_ids;
	const cv::Ptr<cv::aruco::Dictionary> dict =
		cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
	cv::aruco::detectMarkers(gray, dict, marker_corners, marker_ids,
		make_detector_params());
	if (marker_ids.empty()) {
		return out;
	}
	const cv::Ptr<cv::aruco::CharucoBoard> board = make_board();
	cv::aruco::interpolateCornersCharuco(marker_corners, marker_ids, gray, board,
		out.charuco_corners, out.charuco_ids);
	out.corner_count = static_cast<int>(out.charuco_ids.size());
	return out;
}

bool CharucoBoardSpec::match_view(const CharucoDetectResult& det,
	std::vector<cv::Point3f>& obj_points,
	std::vector<cv::Point2f>& img_points)
{
	obj_points.clear();
	img_points.clear();
	if (det.corner_count < kMinCorners) {
		return false;
	}
	const cv::Ptr<cv::aruco::CharucoBoard> board = make_board();
	cv::Mat obj_mat, img_mat;
	board->matchImagePoints(det.charuco_corners, det.charuco_ids, obj_mat, img_mat);
	if (obj_mat.empty() || img_mat.empty()) {
		return false;
	}
	for (int i = 0; i < obj_mat.rows; ++i) {
		obj_points.emplace_back(
			obj_mat.at<float>(i, 0),
			obj_mat.at<float>(i, 1),
			obj_mat.at<float>(i, 2));
		img_points.emplace_back(img_mat.at<float>(i, 0), img_mat.at<float>(i, 1));
	}
	return obj_points.size() >= static_cast<size_t>(kMinCorners);
}
