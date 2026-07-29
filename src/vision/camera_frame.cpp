#include "vision/camera_frame.h"

CameraFrame make_camera_frame(const Pylon::CGrabResultPtr& result,
	uint64_t frame_index, int64_t host_time_ns)
{
	CameraFrame frame;
	frame.frame_index = frame_index;
	frame.host_time_ns = host_time_ns;
	if (!result->GrabSucceeded()) {
		return frame;
	}
	frame.grab_ok = true;
	frame.camera_ts_ticks = result->GetTimeStamp();
	frame.image_size = cv::Size(
		static_cast<int>(result->GetWidth()),
		static_cast<int>(result->GetHeight()));
	frame.mono8 = cv::Mat(frame.image_size.height, frame.image_size.width,
		CV_8UC1, result->GetBuffer());
	return frame;
}
