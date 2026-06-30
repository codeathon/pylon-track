#pragma once

#include <atomic>
#include <thread>

class FerretTracker;

// Runs imshow / waitKey on a dedicated thread so pylon grab and stdout stay responsive.
class DisplayThread {
public:
	void start(FerretTracker* tracker, std::atomic<bool>* app_running);
	void stop();

private:
	void run();

	FerretTracker* tracker_ = nullptr;
	std::atomic<bool>* app_running_ = nullptr;
	std::atomic<bool> display_running_{false};
	std::thread thread_;
};
