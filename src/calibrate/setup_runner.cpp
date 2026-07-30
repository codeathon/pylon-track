#include "calibrate/setup_runner.h"

#include <filesystem>
#include <iostream>

#include <opencv2/core.hpp>

#include "calibrate/arena_calibrator.h"
#include "calibrate/camera_calibrator.h"
#include "calibrate/labjack_calibrator.h"
#include "calibrate/odrive_calibrator.h"
#include "calibrate/setup_util.h"
#include "camera/camera_settings.h"
#include "experiment/arena_config.h"
#include "log/logger.h"

namespace fs = std::filesystem;

SetupRunner::SetupRunner(SetupOptions opts) : opts_(std::move(opts)) {}

bool SetupRunner::should_run(SetupStep step) const {
	if (opts_.only == SetupStep::All) {
		return true;
	}
	return opts_.only == step;
}

int SetupRunner::run() {
	const std::string config_path = resolve_arena_config_path(
		opts_.argv0, opts_.arena_config_path);
	if (config_path.empty()) {
		log_error("setup", "No arena config — pass --config <path>");
		return 1;
	}

	ArenaExperimentConfig cfg;
	if (!load_arena_experiment_config(config_path, cfg)) {
		return 1;
	}

	const std::string calib_path = resolve_calib_output_path(
		opts_.argv0, opts_.calib_path);
	const std::string camera_config_path = resolve_camera_config_path(
		opts_.argv0, opts_.camera_config_path);
	CameraSettings camera_settings;
	cv::Size expected_size;
	if (load_camera_config(camera_config_path, camera_settings)) {
		expected_size = cv::Size(camera_settings.width, camera_settings.height);
	}

	constexpr int kTotal = 4;
	int step_num = 0;
	bool ok = true;

	if (should_run(SetupStep::Arena)) {
		print_setup_banner("arena masks", ++step_num, kTotal);
		ArenaCalibrator arena(opts_);
		if (!arena.run(config_path, cfg)) {
			log_error("setup", "Arena step failed");
			ok = false;
		}
	}

	if (ok && should_run(SetupStep::Camera)) {
		print_setup_banner("camera lens", ++step_num, kTotal);
		CameraCalibrator camera(opts_);
		if (!camera.run(calib_path, expected_size)) {
			log_error("setup", "Camera step failed");
			ok = false;
		}
	}

	if (ok && should_run(SetupStep::ODrive)) {
		print_setup_banner("ODrive chain", ++step_num, kTotal);
		ODriveCalibrator odrive(opts_);
		if (!odrive.run(config_path, cfg)) {
			log_error("setup", "ODrive step failed");
			ok = false;
		}
	}

	if (ok && should_run(SetupStep::LabJack)) {
		print_setup_banner("LabJack shuttle motor", ++step_num, kTotal);
		LabjackCalibrator labjack(opts_);
		if (!labjack.run(cfg)) {
			log_error("setup", "LabJack step failed");
			ok = false;
		}
	}

	if (ok) {
		std::cout << "\n=== Setup complete — run: arena_experiment run --config "
			<< config_path << " ===\n";
		return 0;
	}
	return 1;
}
