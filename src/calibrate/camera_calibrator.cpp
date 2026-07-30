#include "calibrate/camera_calibrator.h"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

#include "calibrate/charuco_board.h"
#include "calibrate/setup_util.h"
#include "camera/camera_calib.h"
#include "camera/camera_config.h"
#include "camera/camera_settings.h"
#include "log/logger.h"

using namespace Pylon;
namespace fs = std::filesystem;

namespace {

constexpr int kMinFrames = 12;
constexpr double kUndistortAlpha = 0.25;

bool run_standard_calib(const std::vector<std::vector<cv::Point3f>>& obj_points,
	const std::vector<std::vector<cv::Point2f>>& img_points,
	const cv::Size& img_size, CameraCalib& out)
{
	if (obj_points.size() < 5) {
		return false;
	}
	cv::Mat k_mat, dist;
	std::vector<cv::Mat> rvecs, tvecs;
	const double rms = cv::calibrateCamera(obj_points, img_points, img_size,
		k_mat, dist, rvecs, tvecs, cv::CALIB_RATIONAL_MODEL);
	out.model = CalibModel::Standard;
	out.camera_matrix = k_mat;
	out.dist_coeffs = dist;
	out.rms = rms;
	out.undistort_alpha = kUndistortAlpha;
	out.image_size = img_size;
	return true;
}

bool run_fisheye_calib(const std::vector<std::vector<cv::Point3f>>& obj_points,
	const std::vector<std::vector<cv::Point2f>>& img_points,
	const cv::Size& img_size, CameraCalib& out)
{
	std::vector<std::vector<cv::Point3f>> obj3d;
	std::vector<std::vector<cv::Point2f>> img2d;
	for (size_t i = 0; i < obj_points.size(); ++i) {
		if (obj_points[i].size() >= 8) {
			obj3d.push_back(obj_points[i]);
			img2d.push_back(img_points[i]);
		}
	}
	if (obj3d.size() < 5) {
		return false;
	}
	cv::Mat k_mat = cv::Mat::eye(3, 3, CV_64F);
	k_mat.at<double>(0, 0) = std::max(img_size.width, img_size.height) * 0.85;
	k_mat.at<double>(1, 1) = k_mat.at<double>(0, 0);
	k_mat.at<double>(0, 2) = img_size.width * 0.5;
	k_mat.at<double>(1, 2) = img_size.height * 0.5;
	cv::Mat dist = cv::Mat::zeros(4, 1, CV_64F);
	const int flags = cv::fisheye::CALIB_RECOMPUTE_EXTRINSIC | cv::fisheye::CALIB_FIX_SKEW;
	const cv::TermCriteria criteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 100, 1e-6);
	double rms = 0.0;
	try {
		rms = cv::fisheye::calibrate(obj3d, img2d, img_size, k_mat, dist,
			cv::noArray(), cv::noArray(), flags, criteria);
	} catch (const cv::Exception& e) {
		log_error("setup", std::string("fisheye calibrate failed: ") + e.what());
		return false;
	}
	out.model = CalibModel::Fisheye;
	out.camera_matrix = k_mat;
	out.dist_coeffs = dist;
	out.rms = rms;
	out.undistort_alpha = kUndistortAlpha;
	out.image_size = img_size;
	return true;
}

int count_png_frames(const std::string& dir) {
	int n = 0;
	if (!fs::is_directory(dir)) {
		return 0;
	}
	for (const auto& entry : fs::directory_iterator(dir)) {
		if (entry.path().extension() == ".png") {
			++n;
		}
	}
	return n;
}

} // namespace

CameraCalibrator::CameraCalibrator(const SetupOptions& opts) : opts_(opts) {}

bool CameraCalibrator::capture_frames(const std::string& camera_config_path,
	cv::Size& img_size)
{
	if (!opts_.display) {
		log_error("setup", "Camera capture requires --display");
		return false;
	}
	CameraSettings settings;
	if (!load_camera_config(camera_config_path, settings)) {
		return false;
	}
	fs::create_directories(opts_.calib_frames_dir);
	PylonInitialize();
	bool ok = false;
	try {
		CBaslerUniversalInstantCamera camera(
			CTlFactory::GetInstance().CreateFirstDevice());
		configure_camera(camera, settings);
		camera.StartGrabbing(GrabStrategy_LatestImageOnly);
		CImageFormatConverter converter;
		converter.OutputPixelFormat = PixelType_Mono8;
		cv::namedWindow("camera_setup", cv::WINDOW_NORMAL);
		int saved = count_png_frames(opts_.calib_frames_dir);
		std::cout << "SPACE=save frame, q=quit. Need ~20 frames with ChArUco visible.\n";
		while (camera.IsGrabbing()) {
			CGrabResultPtr result;
			if (!camera.RetrieveResult(500, result, TimeoutHandling_Return)) {
				continue;
			}
			if (!result->GrabSucceeded()) {
				continue;
			}
			CPylonImage pylon_image;
			converter.Convert(pylon_image, result);
			cv::Mat gray(result->GetHeight(), result->GetWidth(), CV_8UC1,
				static_cast<uint8_t*>(pylon_image.GetBuffer()));
			img_size = gray.size();
			const CharucoDetectResult det = CharucoBoardSpec::detect(gray);
			cv::Mat disp;
			cv::cvtColor(gray, disp, cv::COLOR_GRAY2BGR);
			if (det.corner_count > 0) {
				cv::aruco::drawDetectedCornersCharuco(
					disp, det.charuco_corners, det.charuco_ids, cv::Scalar(0, 255, 0));
			}
			cv::putText(disp, "saved:" + std::to_string(saved)
				+ " corners:" + std::to_string(det.corner_count),
				cv::Point(20, 40), cv::FONT_HERSHEY_SIMPLEX, 0.9, cv::Scalar(0, 255, 0), 2);
			cv::imshow("camera_setup", disp);
			const int key = cv::waitKey(1) & 0xFF;
			if (key == 'q') {
				break;
			}
			if (key == ' ' && det.corner_count >= 8) {
				char name[128];
				std::snprintf(name, sizeof(name), "%s/frame_%03d.png",
					opts_.calib_frames_dir.c_str(), saved);
				cv::imwrite(name, gray);
				++saved;
				std::cout << "Saved " << name << '\n';
			}
		}
		camera.StopGrabbing();
		cv::destroyWindow("camera_setup");
		ok = saved >= kMinFrames;
		if (!ok) {
			log_error("setup", "Need at least " + std::to_string(kMinFrames) + " frames");
		}
	} catch (const GenericException& e) {
		log_error("setup", std::string("Camera capture error: ") + e.GetDescription());
	}
	PylonTerminate();
	return ok;
}

bool CameraCalibrator::calibrate_from_frames(const std::string& calib_output_path,
	cv::Size img_size)
{
	std::vector<std::vector<cv::Point3f>> obj_points;
	std::vector<std::vector<cv::Point2f>> img_points;
	int loaded = 0;
	for (const auto& entry : fs::directory_iterator(opts_.calib_frames_dir)) {
		if (entry.path().extension() != ".png") {
			continue;
		}
		const cv::Mat gray = cv::imread(entry.path().string(), cv::IMREAD_GRAYSCALE);
		if (gray.empty()) {
			continue;
		}
		img_size = gray.size();
		const CharucoDetectResult det = CharucoBoardSpec::detect(gray);
		std::vector<cv::Point3f> obj;
		std::vector<cv::Point2f> img;
		if (!CharucoBoardSpec::match_view(det, obj, img)) {
			continue;
		}
		obj_points.push_back(obj);
		img_points.push_back(img);
		++loaded;
	}
	if (loaded < kMinFrames) {
		log_error("setup", "Not enough valid calibration views");
		return false;
	}
	log_info("setup", "Calibrating from " + std::to_string(loaded) + " views");
	CameraCalib standard;
	CameraCalib fisheye;
	const bool std_ok = run_standard_calib(obj_points, img_points, img_size, standard);
	const bool fish_ok = run_fisheye_calib(obj_points, img_points, img_size, fisheye);
	CameraCalib chosen;
	if (std_ok && fish_ok) {
		chosen = (fisheye.rms < standard.rms) ? fisheye : standard;
	} else if (fish_ok) {
		chosen = fisheye;
	} else if (std_ok) {
		chosen = standard;
	} else {
		return false;
	}
	log_info("setup", std::string("Selected model RMS=") + std::to_string(chosen.rms));
	return save_camera_calib(calib_output_path, chosen);
}

bool CameraCalibrator::run(const std::string& calib_output_path, cv::Size expected_size) {
	(void)expected_size;
	const std::string camera_config_path = resolve_camera_config_path(
		opts_.argv0, opts_.camera_config_path);
	cv::Size img_size;
	if (!opts_.skip_interactive) {
		if (!capture_frames(camera_config_path, img_size)) {
			return false;
		}
	}
	return calibrate_from_frames(calib_output_path, img_size);
}
