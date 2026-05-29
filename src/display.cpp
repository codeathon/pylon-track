#include "display.h"
#include "ferret_tracker.h"
#include "logger.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <opencv2/highgui.hpp>

namespace {

static constexpr int kWindowPollMs = 1;
static constexpr auto kMinFrameInterval = std::chrono::milliseconds(33); // ~30 Hz UI

cv::Mat render_overlay(const DisplaySnapshot& snap) {
	cv::Mat vis;
	cv::cvtColor(snap.frame, vis, cv::COLOR_GRAY2BGR);

	const cv::Scalar ferret_color(0, 255, 0);
	const cv::Scalar prey_color(255, 255, 0);

	for (size_t i = 0; i < snap.contours.size(); ++i) {
		const cv::Scalar color = (i == 0) ? ferret_color : prey_color;
		cv::drawContours(vis, snap.contours, static_cast<int>(i), color, 2);
	}

	auto draw_marker = [&](const TrackState& state, const char* label, const cv::Scalar& color) {
		if (!state.valid) {
			return;
		}
		cv::Point center(static_cast<int>(state.pos_px.x), static_cast<int>(state.pos_px.y));
		cv::circle(vis, center, 8, color, 2);
		cv::putText(vis, label, center + cv::Point(10, -10),
			cv::FONT_HERSHEY_SIMPLEX, 0.7, color, 2);
	};

	draw_marker(snap.ferret, "Ferret", ferret_color);
	draw_marker(snap.prey, "Prey", prey_color);

	if (snap.ferret.valid && snap.prey.valid) {
		cv::Point f(static_cast<int>(snap.ferret.pos_px.x), static_cast<int>(snap.ferret.pos_px.y));
		cv::Point p(static_cast<int>(snap.prey.pos_px.x), static_cast<int>(snap.prey.pos_px.y));
		cv::line(vis, f, p, cv::Scalar(0, 165, 255), 2);

		float dx = snap.ferret.pos_mm.x - snap.prey.pos_mm.x;
		float dy = snap.ferret.pos_mm.y - snap.prey.pos_mm.y;
		float dist_mm = std::sqrt(dx * dx + dy * dy);
		char buf[64];
		std::snprintf(buf, sizeof(buf), "Dist: %.0f mm", dist_mm);
		cv::putText(vis, buf, cv::Point(20, 40),
			cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
	}

	if (snap.warmup) {
		cv::putText(vis, "Warming up background (keep arena empty)...",
			cv::Point(20, vis.rows - 30), cv::FONT_HERSHEY_SIMPLEX, 0.8,
			cv::Scalar(0, 200, 255), 2);
	}

	return vis;
}

} // namespace

void DisplayThread::start(FerretTracker* tracker, std::atomic<bool>* app_running) {
	if (display_running_.load()) {
		return;
	}
	tracker_ = tracker;
	app_running_ = app_running;
	display_running_.store(true);
	thread_ = std::thread(&DisplayThread::run, this);
}

void DisplayThread::stop() {
	if (!display_running_.load()) {
		return;
	}
	display_running_.store(false);
	if (thread_.joinable()) {
		thread_.join();
	}
	cv::destroyAllWindows();
	log_info("display", "Display thread stopped");
	tracker_ = nullptr;
	app_running_ = nullptr;
}

void DisplayThread::run() {
	log_info("display", "Display thread started");
	cv::namedWindow("pylon-track", cv::WINDOW_NORMAL);
	auto last_show = std::chrono::steady_clock::now();
	int empty_snapshot_polls = 0;
	bool warned_no_snapshot = false;

	while (display_running_.load() && app_running_ && app_running_->load()) {
		const auto now = std::chrono::steady_clock::now();
		if (now - last_show < kMinFrameInterval) {
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			continue;
		}

		DisplaySnapshot snap;
		if (!tracker_ || !tracker_->get_display_snapshot(snap)) {
			++empty_snapshot_polls;
			if (!warned_no_snapshot && empty_snapshot_polls > 100) {
				log_warn("display", "No display snapshot yet — waiting for camera frames");
				warned_no_snapshot = true;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(5));
			continue;
		}
		empty_snapshot_polls = 0;

		cv::Mat vis = render_overlay(snap);
		cv::imshow("pylon-track", vis);
		last_show = now;

		const int key = cv::waitKey(kWindowPollMs) & 0xFF;
		if (key == 'q' || key == 'Q' || key == 27) {
			app_running_->store(false);
			break;
		}
	}
}
