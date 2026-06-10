#pragma once

#include <vector>

#include <opencv2/opencv.hpp>

// Per-image quality metrics for the parameter sweep, chosen to map directly
// onto Basler's image-optimization guidance (docs.baslerweb.com):
//   mean_gray        — brightness; target ~50-70% of range (128-180 for Mono8)
//   stddev           — global contrast proxy
//   clipped_*_pct    — % pixels at 0 / 255; clipping destroys blob contours
//   laplacian_var    — sharpness/noise proxy; drops with motion blur and
//                      defocus, rises with gain noise (compare across sweeps)
struct ImageMetrics {
	double mean_gray = 0.0;
	double stddev = 0.0;
	double clipped_low_pct = 0.0;
	double clipped_high_pct = 0.0;
	double laplacian_var = 0.0;
};

// Computes metrics for one Mono8 frame.
ImageMetrics compute_image_metrics(const cv::Mat& mono8);

// Element-wise mean of per-frame metrics (sweep reports one row per value).
ImageMetrics average_metrics(const std::vector<ImageMetrics>& samples);
