#pragma once

#include <atomic>
#include <thread>

class CameraTrackingService;

// Runs imshow / waitKey on a dedicated thread so pylon grab stays responsive.
class DisplayThread {
public:
	void start(CameraTrackingService* tracker, std::atomic<bool>* app_running);
	void stop();

private:
	void run();

	CameraTrackingService* tracker_ = nullptr;
	std::atomic<bool>* app_running_ = nullptr;
	std::atomic<bool> display_running_{false};
	std::thread thread_;
};
