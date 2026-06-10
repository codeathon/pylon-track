#include "image_metrics.h"

ImageMetrics compute_image_metrics(const cv::Mat& mono8) {
	ImageMetrics m;

	cv::Scalar mean, stddev;
	cv::meanStdDev(mono8, mean, stddev);
	m.mean_gray = mean[0];
	m.stddev = stddev[0];

	const double total = static_cast<double>(mono8.total());
	// Clipped pixels carry no gradient information — contours break there.
	m.clipped_low_pct = 100.0 * cv::countNonZero(mono8 == 0) / total;
	m.clipped_high_pct = 100.0 * cv::countNonZero(mono8 == 255) / total;

	// Variance of the Laplacian: standard single-image sharpness measure.
	cv::Mat lap;
	cv::Laplacian(mono8, lap, CV_64F);
	cv::Scalar lap_mean, lap_stddev;
	cv::meanStdDev(lap, lap_mean, lap_stddev);
	m.laplacian_var = lap_stddev[0] * lap_stddev[0];

	return m;
}

ImageMetrics average_metrics(const std::vector<ImageMetrics>& samples) {
	ImageMetrics avg;
	if (samples.empty()) {
		return avg;
	}
	for (const ImageMetrics& s : samples) {
		avg.mean_gray += s.mean_gray;
		avg.stddev += s.stddev;
		avg.clipped_low_pct += s.clipped_low_pct;
		avg.clipped_high_pct += s.clipped_high_pct;
		avg.laplacian_var += s.laplacian_var;
	}
	const double n = static_cast<double>(samples.size());
	avg.mean_gray /= n;
	avg.stddev /= n;
	avg.clipped_low_pct /= n;
	avg.clipped_high_pct /= n;
	avg.laplacian_var /= n;
	return avg;
}
