#pragma once

#include <opencv2/core.hpp>
#include <vector>

// Polygon region in image pixel coordinates (arena floor, pulley zones, etc.).
struct PolygonRegion {
	std::vector<cv::Point> points;
};

// Vision mask config — loaded from arena_experiment.json "vision" section.
struct ArenaMaskConfig {
	std::vector<PolygonRegion> ignore_regions;
	std::vector<cv::Point> track_roi;
	bool has_track_roi = false;
};

// Zeros foreground mask inside ignore_regions; optionally clips to track_roi.
class ArenaMask {
public:
	explicit ArenaMask(const ArenaMaskConfig& cfg);

	bool enabled() const { return enabled_; }
	void apply(cv::Mat& foreground_mask) const;

private:
	bool enabled_ = false;
	std::vector<PolygonRegion> ignore_regions_;
	std::vector<cv::Point> track_roi_;
	bool has_track_roi_ = false;
};
