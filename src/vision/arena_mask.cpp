#include "vision/arena_mask.h"

#include <opencv2/imgproc.hpp>

ArenaMask::ArenaMask(const ArenaMaskConfig& cfg)
	: ignore_regions_(cfg.ignore_regions)
	, track_roi_(cfg.track_roi)
	, has_track_roi_(cfg.has_track_roi && cfg.track_roi.size() >= 3)
{
	enabled_ = !ignore_regions_.empty() || has_track_roi_;
}

void ArenaMask::apply(cv::Mat& foreground_mask) const {
	if (!enabled_ || foreground_mask.empty()) {
		return;
	}

	// Clip to arena floor ROI first so MOG2 blobs outside the floor are dropped.
	if (has_track_roi_) {
		cv::Mat roi_mask = cv::Mat::zeros(foreground_mask.size(), CV_8UC1);
		const std::vector<std::vector<cv::Point>> roi_poly = {track_roi_};
		cv::fillPoly(roi_mask, roi_poly, 255);
		cv::bitwise_and(foreground_mask, roi_mask, foreground_mask);
	}

	// Zero pulley/chain exclusion zones so rig hardware never becomes a track candidate.
	for (const auto& region : ignore_regions_) {
		if (region.points.size() < 3) {
			continue;
		}
		const std::vector<std::vector<cv::Point>> poly = {region.points};
		cv::fillPoly(foreground_mask, poly, 0);
	}
}
