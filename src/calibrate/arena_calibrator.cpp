#include "calibrate/arena_calibrator.h"

#include <iostream>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

#include "calibrate/setup_util.h"
#include "camera/camera_config.h"
#include "camera/camera_settings.h"
#include "experiment/arena_config.h"
#include "log/logger.h"

using namespace Pylon;

namespace {

struct ArenaUiState {
	ArenaMaskConfig* mask = nullptr;
	std::vector<cv::Point> current_poly;
	bool drawing_ignore = true;
	bool dirty = false;
};

void draw_overlays(cv::Mat& vis, const ArenaUiState& ui) {
	for (const auto& region : ui.mask->ignore_regions) {
		if (region.points.size() >= 2) {
			const cv::Point* pts = region.points.data();
			const int n = static_cast<int>(region.points.size());
			cv::polylines(vis, &pts, &n, 1, true, cv::Scalar(0, 0, 255), 2);
		}
	}
	if (ui.mask->has_track_roi && ui.mask->track_roi.size() >= 2) {
		const cv::Point* pts = ui.mask->track_roi.data();
		const int n = static_cast<int>(ui.mask->track_roi.size());
		cv::polylines(vis, &pts, &n, 1, true, cv::Scalar(0, 255, 0), 2);
	}
	if (ui.current_poly.size() >= 2) {
		const cv::Point* pts = ui.current_poly.data();
		const int n = static_cast<int>(ui.current_poly.size());
		const cv::Scalar color = ui.drawing_ignore
			? cv::Scalar(0, 128, 255) : cv::Scalar(0, 255, 128);
		cv::polylines(vis, &pts, &n, 1, false, color, 2);
	}
	cv::putText(vis, "i=ignore r=roi u=undo s=save q=quit LMB=vertex RMB=close",
		cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.55, cv::Scalar(0, 255, 0), 1);
}

void close_polygon(ArenaUiState& ui) {
	if (ui.current_poly.size() < 3) {
		ui.current_poly.clear();
		return;
	}
	if (ui.drawing_ignore) {
		PolygonRegion region;
		region.points = ui.current_poly;
		ui.mask->ignore_regions.push_back(region);
	} else {
		ui.mask->track_roi = ui.current_poly;
		ui.mask->has_track_roi = true;
	}
	ui.current_poly.clear();
	ui.dirty = true;
}

void on_mouse(int event, int x, int y, int /*flags*/, void* userdata) {
	auto* ui = static_cast<ArenaUiState*>(userdata);
	if (event == cv::EVENT_LBUTTONDOWN) {
		ui->current_poly.emplace_back(x, y);
	} else if (event == cv::EVENT_RBUTTONDOWN) {
		close_polygon(*ui);
	}
}

} // namespace

ArenaCalibrator::ArenaCalibrator(const SetupOptions& opts) : opts_(opts) {}

bool ArenaCalibrator::run(const std::string& config_path, ArenaExperimentConfig& cfg) {
	if (!opts_.display) {
		log_error("setup", "Arena setup requires --display and DISPLAY");
		return false;
	}
	const std::string camera_config_path = resolve_camera_config_path(
		opts_.argv0, opts_.camera_config_path);
	CameraSettings camera_settings;
	if (!load_camera_config(camera_config_path, camera_settings)) {
		return false;
	}

	PylonInitialize();
	bool ok = false;
	try {
		CBaslerUniversalInstantCamera camera(
			CTlFactory::GetInstance().CreateFirstDevice());
		configure_camera(camera, camera_settings);
		camera.StartGrabbing(GrabStrategy_LatestImageOnly);

		ArenaUiState ui;
		ui.mask = &cfg.vision.mask;
		cv::namedWindow("arena_setup", cv::WINDOW_NORMAL);
		cv::setMouseCallback("arena_setup", on_mouse, &ui);

		std::cout << "Draw ignore regions (i) and track ROI (r). "
			<< "LMB=vertex, RMB=close polygon.\n";

		CImageFormatConverter converter;
		converter.OutputPixelFormat = PixelType_Mono8;

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
			const cv::Mat gray(result->GetHeight(), result->GetWidth(), CV_8UC1,
				static_cast<uint8_t*>(pylon_image.GetBuffer()));

			cv::Mat vis;
			cv::cvtColor(gray, vis, cv::COLOR_GRAY2BGR);
			draw_overlays(vis, ui);
			cv::imshow("arena_setup", vis);

			const int key = cv::waitKey(1) & 0xFF;
			if (key == 'q') {
				break;
			}
			if (key == 'i') {
				ui.drawing_ignore = true;
			}
			if (key == 'r') {
				ui.drawing_ignore = false;
			}
			if (key == 'u') {
				if (!ui.current_poly.empty()) {
					ui.current_poly.pop_back();
				} else if (!ui.mask->ignore_regions.empty()) {
					ui.mask->ignore_regions.pop_back();
					ui.dirty = true;
				}
			}
			if (key == 's') {
				save_arena_vision_masks(config_path, cfg.vision.mask);
			}
		}
		camera.StopGrabbing();
		ok = ui.dirty ? save_arena_vision_masks(config_path, cfg.vision.mask) : true;
		cv::destroyWindow("arena_setup");
	} catch (const GenericException& e) {
		log_error("setup", std::string("Arena setup camera error: ") + e.GetDescription());
		ok = false;
	}
	PylonTerminate();
	return ok;
}
