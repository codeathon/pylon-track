#include "measurement_run.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <mutex>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>

#include "log/logger.h"
#include "test_session.h"

namespace fs = std::filesystem;
using SteadyClock = std::chrono::steady_clock;

namespace {

// Wraps the production tracker to time grab -> distance computation and log
// every measured frame. Row writing happens AFTER the t1 stamp so CSV I/O is
// excluded from the reported latency.
class TimedLatencyTracker : public FerretTracker {
public:
	TimedLatencyTracker(const MeasurementOptions& opts, CsvWriter& frames_csv)
		: FerretTracker(/*enable_display=*/opts.annotate_every_n > 0,
			static_cast<int>(opts.warmup_s * FPS),
			opts.gsd_mm_px)
		, opts_(opts)
		, frames_csv_(frames_csv)
		, warmup_frames_count_(static_cast<uint64_t>(opts.warmup_s * FPS))
		, start_(SteadyClock::now())
	{}

	void OnImageGrabbed(Pylon::CInstantCamera& camera,
		const Pylon::CGrabResultPtr& result) override
	{
		if (!result->GrabSucceeded()) {
			return;
		}
		const auto t0 = SteadyClock::now();
		FerretTracker::OnImageGrabbed(camera, result);

		// Distance between the two tracked objects — the measurement whose
		// end-to-end latency the suite characterizes.
		float distance_mm = -1.0f;
		if (ferret.valid && prey.valid) {
			const float dx = ferret.pos_mm.x - prey.pos_mm.x;
			const float dy = ferret.pos_mm.y - prey.pos_mm.y;
			distance_mm = std::sqrt(dx * dx + dy * dy);
		}
		const auto t1 = SteadyClock::now();

		if (frame_count_ <= warmup_frames_count_) {
			return; // warmup frames train the background model, not the CSV
		}
		record_frame(result, distance_mm, t0, t1);
	}

	std::vector<long> latencies_us() const {
		std::lock_guard<std::mutex> lock(stats_mutex_);
		return latencies_us_;
	}

	uint64_t measured_frames() const {
		std::lock_guard<std::mutex> lock(stats_mutex_);
		return measured_frames_;
	}

	uint64_t valid_pair_frames() const {
		std::lock_guard<std::mutex> lock(stats_mutex_);
		return valid_pair_frames_;
	}

private:
	void record_frame(const Pylon::CGrabResultPtr& result, float distance_mm,
		SteadyClock::time_point t0, SteadyClock::time_point t1)
	{
		const long latency_us = std::chrono::duration_cast<
			std::chrono::microseconds>(t1 - t0).count();
		const long host_us = std::chrono::duration_cast<
			std::chrono::microseconds>(t0 - start_).count();

		std::vector<std::string> row = {
			std::to_string(frame_count_),
			std::to_string(result->GetTimeStamp()),
			std::to_string(host_us),
		};
		if (opts_.height_cm >= 0.0) {
			row.push_back(csv_num(opts_.height_cm, 1));
		}
		const std::vector<std::string> track_cells = {
			csv_num(ferret.speed_mm_s, 1), csv_num(prey.speed_mm_s, 1),
			csv_num(ferret.pos_px.x, 1), csv_num(ferret.pos_px.y, 1),
			csv_num(prey.pos_px.x, 1), csv_num(prey.pos_px.y, 1),
			csv_num(ferret.pos_mm.x, 1), csv_num(ferret.pos_mm.y, 1),
			csv_num(prey.pos_mm.x, 1), csv_num(prey.pos_mm.y, 1),
			csv_num(distance_mm, 1),
			std::to_string(latency_us),
			ferret.valid ? "1" : "0",
			prey.valid ? "1" : "0",
		};
		row.insert(row.end(), track_cells.begin(), track_cells.end());
		frames_csv_.write_row(row);

		std::lock_guard<std::mutex> lock(stats_mutex_);
		latencies_us_.push_back(latency_us);
		++measured_frames_;
		if (ferret.valid && prey.valid) {
			++valid_pair_frames_;
		}
	}

	const MeasurementOptions& opts_;
	CsvWriter& frames_csv_;
	const uint64_t warmup_frames_count_;
	const SteadyClock::time_point start_;

	mutable std::mutex stats_mutex_;
	std::vector<long> latencies_us_;
	uint64_t measured_frames_ = 0;
	uint64_t valid_pair_frames_ = 0;
};

std::vector<std::string> frames_csv_header(bool with_height) {
	std::vector<std::string> header = {
		"frame_index", "camera_ts_ticks", "host_time_us",
	};
	if (with_height) {
		header.push_back("height_cm");
	}
	const std::vector<std::string> track_cols = {
		"speed1_mm_s", "speed2_mm_s",
		"c1_x_px", "c1_y_px", "c2_x_px", "c2_y_px",
		"c1_x_mm", "c1_y_mm", "c2_x_mm", "c2_y_mm",
		"distance_mm", "latency_us", "valid1", "valid2",
	};
	header.insert(header.end(), track_cols.begin(), track_cols.end());
	return header;
}

// Draws contours, bounding boxes and px² areas so the operator can judge
// whether objects still resolve at the current mount height.
void save_annotated_still(const DisplaySnapshot& snap, const std::string& dir) {
	cv::Mat bgr;
	cv::cvtColor(snap.frame, bgr, cv::COLOR_GRAY2BGR);
	cv::drawContours(bgr, snap.contours, -1, {0, 255, 0}, 1);
	for (const auto& contour : snap.contours) {
		const cv::Rect box = cv::boundingRect(contour);
		const double area = cv::contourArea(contour);
		cv::rectangle(bgr, box, {0, 0, 255}, 1);
		// 200 px² is the production tracker's noise floor — label area so the
		// operator can see margin against it at each candidate height.
		cv::putText(bgr, std::to_string(static_cast<int>(area)) + "px2",
			{box.x, std::max(box.y - 5, 12)},
			cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 0, 255}, 1);
	}
	char name[64];
	std::snprintf(name, sizeof(name), "still_f%08llu.png",
		static_cast<unsigned long long>(snap.frame_count));
	cv::imwrite((fs::path(dir) / name).string(), bgr);
}

void write_summary(const std::string& session_dir, const MeasurementOptions& opts,
	TimedLatencyTracker& tracker, double elapsed_s)
{
	std::vector<long> lat = tracker.latencies_us();
	std::sort(lat.begin(), lat.end());

	double mean = 0.0;
	for (long v : lat) mean += v;
	mean = lat.empty() ? 0.0 : mean / lat.size();
	const auto pct = [&lat](double p) -> long {
		if (lat.empty()) return 0;
		const size_t idx = std::min(lat.size() - 1,
			static_cast<size_t>(p * lat.size()));
		return lat[idx];
	};

	const uint64_t frames = tracker.measured_frames();
	CsvWriter summary((fs::path(session_dir) / "summary.csv").string(), {
		"frames", "duration_s", "achieved_fps", "valid_pair_pct",
		"latency_mean_us", "latency_p50_us", "latency_p95_us", "latency_max_us",
		"warmup_s", "gsd_mm_px", "height_cm",
	});
	summary.write_row({
		std::to_string(frames),
		csv_num(elapsed_s, 2),
		csv_num(elapsed_s > 0.0 ? frames / elapsed_s : 0.0, 1),
		csv_num(frames > 0 ? 100.0 * tracker.valid_pair_frames() / frames : 0.0, 1),
		csv_num(mean, 1),
		std::to_string(pct(0.50)),
		std::to_string(pct(0.95)),
		std::to_string(lat.empty() ? 0 : lat.back()),
		csv_num(opts.warmup_s, 1),
		csv_num(opts.gsd_mm_px, 4),
		csv_num(opts.height_cm, 1),
	});
}

} // namespace

int run_measurement(Pylon::CBaslerUniversalInstantCamera& camera,
	const MeasurementOptions& opts,
	const std::atomic<bool>& keep_running)
{
	CsvWriter frames_csv((fs::path(opts.session_dir) / "frames.csv").string(),
		frames_csv_header(opts.height_cm >= 0.0));

	std::string stills_dir;
	if (opts.annotate_every_n > 0) {
		stills_dir = (fs::path(opts.session_dir) / "stills").string();
		fs::create_directories(stills_dir);
	}

	TimedLatencyTracker tracker(opts, frames_csv);
	camera.RegisterImageEventHandler(&tracker,
		Pylon::RegistrationMode_Append, Pylon::Cleanup_None);

	log_info("measure", "Warming up background model for "
		+ csv_num(opts.warmup_s, 0) + "s — keep arena empty");
	camera.StartGrabbing(Pylon::GrabStrategy_LatestImageOnly,
		Pylon::GrabLoop_ProvidedByInstantCamera);

	const auto start = SteadyClock::now();
	const double total_s = opts.warmup_s + opts.duration_s;
	uint64_t last_still_frame = 0;
	double measured_elapsed_s = 0.0;

	while (keep_running.load()) {
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
		const double elapsed = std::chrono::duration<double>(
			SteadyClock::now() - start).count();
		if (elapsed >= total_s) {
			break;
		}
		measured_elapsed_s = std::max(0.0, elapsed - opts.warmup_s);

		// Stills come from the display snapshot path (already thread-safe)
		// so annotation cost never lands on the grab thread.
		DisplaySnapshot snap;
		if (opts.annotate_every_n > 0 && tracker.get_display_snapshot(snap)
			&& !snap.warmup
			&& snap.frame_count >= last_still_frame + opts.annotate_every_n) {
			last_still_frame = snap.frame_count;
			save_annotated_still(snap, stills_dir);
		}
	}

	camera.StopGrabbing();
	camera.DeregisterImageEventHandler(&tracker);

	write_summary(opts.session_dir, opts, tracker, measured_elapsed_s);
	log_info("measure", "Measured " + std::to_string(tracker.measured_frames())
		+ " frames -> " + opts.session_dir);
	return 0;
}
