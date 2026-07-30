#include "calibrate/charuco_board.h"

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/version.hpp>

namespace {

constexpr int kSquaresX = 10;
constexpr int kSquaresY = 7;
constexpr float kSquareLen = 0.035f;
constexpr float kMarkerLen = 0.026f;
constexpr int kMinCorners = 8;

// Ubuntu packages ship OpenCV 4.5/4.6 contrib aruco; 4.7+ changed the API.
cv::Ptr<cv::aruco::Dictionary> make_dictionary() {
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
	return cv::makePtr<cv::aruco::Dictionary>(
		cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250));
#else
	return cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_250);
#endif
}

cv::Ptr<cv::aruco::CharucoBoard> make_board() {
	const cv::Ptr<cv::aruco::Dictionary> dict = make_dictionary();
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
	return cv::makePtr<cv::aruco::CharucoBoard>(
		cv::Size(kSquaresX, kSquaresY), kSquareLen, kMarkerLen, *dict);
#else
	// Ubuntu 4.5/4.6: create(squaresX, squaresY, ...) — not Size overload.
	return cv::aruco::CharucoBoard::create(
		kSquaresX, kSquaresY, kSquareLen, kMarkerLen, dict);
#endif
}

cv::Ptr<cv::aruco::DetectorParameters> make_detector_params() {
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
	auto params = cv::makePtr<cv::aruco::DetectorParameters>();
#else
	cv::Ptr<cv::aruco::DetectorParameters> params =
		cv::aruco::DetectorParameters::create();
#endif
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
	const cv::Ptr<cv::aruco::Dictionary> dict = make_dictionary();
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
#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
	cv::Mat obj_mat, img_mat;
	board->matchImagePoints(det.charuco_corners, det.charuco_ids, obj_mat, img_mat);
	if (obj_mat.empty() || img_mat.empty()) {
		return false;
	}
	for (int i = 0; i < obj_mat.rows; ++i) {
		const cv::Point3f obj = obj_mat.at<cv::Point3f>(i);
		const cv::Point2f img = img_mat.at<cv::Point2f>(i);
		obj_points.push_back(obj);
		img_points.push_back(img);
	}
#else
	// Pre-4.7 CharucoBoard exposes chessboardCorners indexed by charuco id.
	for (size_t i = 0; i < det.charuco_ids.size(); ++i) {
		const int id = det.charuco_ids[i];
		if (id < 0 || id >= static_cast<int>(board->chessboardCorners.size())) {
			continue;
		}
		obj_points.push_back(board->chessboardCorners[id]);
		img_points.push_back(det.charuco_corners[i]);
	}
#endif
	return obj_points.size() >= static_cast<size_t>(kMinCorners);
}
