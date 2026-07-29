#pragma once

#include <cstdint>
#include <opencv2/core.hpp>
#include <pylon/PylonIncludes.h>

// Raw pylon grab — no OpenCV tracking yet. mono8 view is non-owning (valid only
// for the duration of the grab callback).
struct CameraFrame {
	uint64_t frame_index = 0;
	uint64_t camera_ts_ticks = 0;
	int64_t host_time_ns = 0;
	cv::Size image_size;
	cv::Mat mono8;
	bool grab_ok = false;
};

CameraFrame make_camera_frame(const Pylon::CGrabResultPtr& result,
	uint64_t frame_index, int64_t host_time_ns);
