#include "experiment/state_manager.h"

#include <chrono>
#include <cmath>
#include <filesystem>
#include <sstream>

#include <pylon/PylonIncludes.h>
#include <pylon/BaslerUniversalInstantCamera.h>

#include "calibrate/setup_util.h"
#include "camera/camera_calib.h"
#include "camera/camera_config.h"
#include "camera/camera_settings.h"
#include "experiment/arena_config.h"
#include "experiment/session_recorder.h"
#include "experiment/trial_state.h"
#include "log/logger.h"
#include "motor/chase_controller.h"
#include "motor/motion_planner.h"
#include "motor/motor_config.h"
#include "motor/prey_motor.h"
#include "motor/trap_door_motor.h"
#include "tracker/display.h"
#include "vision/camera_tracking_service.h"

using namespace Pylon;

namespace {

constexpr int kStatusLogIntervalMs = 1000;
constexpr int kMainLoopSleepMs = 5;

int64_t host_time_ns() {
	return std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count();
}

} // namespace

const char* component_status_name(ComponentStatus status) {
	switch (status) {
	case ComponentStatus::NotStarted: return "not_started";
	case ComponentStatus::Calibrating: return "calibrating";
	case ComponentStatus::Ready: return "ready";
	case ComponentStatus::Error: return "error";
	}
	return "unknown";
}

const char* experiment_phase_name(ExperimentPhase phase) {
	switch (phase) {
	case ExperimentPhase::Idle: return "idle";
	case ExperimentPhase::Setup: return "setup";
	case ExperimentPhase::Configuring: return "configuring";
	case ExperimentPhase::Streaming: return "streaming";
	case ExperimentPhase::ChaseSession: return "chase_session";
	case ExperimentPhase::Ended: return "ended";
	case ExperimentPhase::Error: return "error";
	}
	return "unknown";
}

ExperimentStateManager::ExperimentStateManager(ExperimentOptions opts) : opts_(std::move(opts)) {}

ExperimentStateManager::~ExperimentStateManager() {
	shutdown();
}

bool ExperimentStateManager::phase_setup() {
	phase_ = ExperimentPhase::Setup;
	const std::string cfg_path = resolve_arena_config_path(
		opts_.argv0, opts_.arena_config_path);
	if (cfg_path.empty()) {
		log_error("experiment", "No arena config found — pass --config <path>");
		phase_ = ExperimentPhase::Error;
		return false;
	}
	if (!load_arena_experiment_config(cfg_path, cfg_)) {
		phase_ = ExperimentPhase::Error;
		return false;
	}

	recorder_ = std::make_unique<SessionRecorder>();
	try {
		recorder_->open_session(opts_.session_dir, "arena_experiment");
	} catch (const std::exception& e) {
		log_error("experiment", std::string("Session recorder failed: ") + e.what());
		phase_ = ExperimentPhase::Error;
		return false;
	}

	trial_fsm_ = std::make_unique<TrialStateMachine>();
	prey_motor_ = std::make_unique<PreyMotor>(prey_motor_from_config(cfg_.motor));
	trap_motor_ = std::make_unique<TrapDoorMotor>(cfg_.trap_door);
	motion_planner_ = std::make_unique<MotionPlanner>();
	// Why: chase flees tick MotionPlanner non-blockingly at 50 Hz.
	chase_controller_ = std::make_unique<ChaseController>(
		*prey_motor_, *motion_planner_, cfg_.chase, cfg_.motor.chain_direction_sign);

	log_info("experiment", "Setup complete — config: " + cfg_path);
	return true;
}

bool ExperimentStateManager::verify_setup_artifacts() {
	if (!opts_.disable_calib) {
		const std::string calib_path = resolve_calib_path(opts_.argv0, opts_.calib_path);
		if (calib_path.empty()) {
			log_error("experiment",
				"calib.npz not found — run: arena_experiment setup --config <path>");
			readiness_.camera = ComponentStatus::Error;
			return false;
		}
		log_info("experiment", "Lens calibration found: " + calib_path);
	}
	if (cfg_.motor.chain_mm_per_motor_turn <= 0.0f) {
		log_error("experiment",
			"motor.chain_mm_per_motor_turn missing — run setup (odrive step)");
		readiness_.motor = ComponentStatus::Error;
		return false;
	}
	if (cfg_.vision.mask.ignore_regions.empty()) {
		log_info("experiment",
			"Warning: no ignore_regions — mark pulleys/chains in setup (arena step)");
	}
	readiness_.camera = ComponentStatus::Ready;
	return true;
}

bool ExperimentStateManager::connect_runtime_hardware() {
	readiness_.motor = ComponentStatus::Calibrating;
	if (!can_interface_up(cfg_.motor.can_interface)) {
		log_error("experiment", can_interface_down_hint(cfg_.motor.can_interface));
		readiness_.motor = ComponentStatus::Error;
	} else if (prey_motor_->connect()) {
		readiness_.motor = ComponentStatus::Ready;
		log_info("experiment", "Prey motor connected");
	} else {
		log_error("experiment", "Prey motor connect failed — chase disabled");
		readiness_.motor = ComponentStatus::Error;
	}

	readiness_.trap_door = ComponentStatus::Calibrating;
	if (trap_motor_->connect()) {
		readiness_.trap_door = ComponentStatus::Ready;
		log_info("experiment", "Trap door ready (" + cfg_.trap_door.backend + ")");
	} else if (cfg_.trap_door.backend == "noop") {
		readiness_.trap_door = ComponentStatus::Ready;
	} else {
		log_error("experiment", "Trap door connect failed");
		readiness_.trap_door = ComponentStatus::Error;
	}
	return true;
}

bool ExperimentStateManager::phase_configuring() {
	phase_ = ExperimentPhase::Configuring;
	try {
		camera_ = std::make_unique<CBaslerUniversalInstantCamera>(
			CTlFactory::GetInstance().CreateFirstDevice());
		log_info("experiment", "Camera: " + std::string(camera_->GetDeviceInfo().GetModelName()));

		const std::string config_path =
			resolve_camera_config_path(opts_.argv0, opts_.camera_config_path);
		CameraSettings camera_settings;
		if (!load_camera_config(config_path, camera_settings)) {
			phase_ = ExperimentPhase::Error;
			return false;
		}
		configure_camera(*camera_, camera_settings);

		std::optional<CameraCalib> calib;
		if (!opts_.disable_calib) {
			const std::string calib_path = resolve_calib_path(opts_.argv0, opts_.calib_path);
			calib = load_camera_calib(calib_path,
				cv::Size(camera_settings.width, camera_settings.height));
			if (!calib) {
				phase_ = ExperimentPhase::Error;
				return false;
			}
		}

		CameraTrackingService::Options track_opts;
		track_opts.warmup_frames = kWarmupFrames;
		track_opts.gsd_mm_px = kGsdMmPx;
		track_opts.fps = kFps;
		track_opts.calib = calib;
		track_opts.mask = cfg_.vision.mask;
		track_opts.vision = cfg_.vision;
		track_opts.enable_display = opts_.enable_display;

		tracking_ = std::make_unique<CameraTrackingService>(track_opts);
		tracking_->set_trial_fsm(trial_fsm_.get());
		tracking_->set_session_recorder(recorder_.get());
		camera_->RegisterImageEventHandler(tracking_.get(),
			RegistrationMode_Append, Cleanup_None);

		connect_runtime_hardware();

		trial_fsm_->begin_warmup();
		camera_->StartGrabbing(GrabStrategy_LatestImageOnly,
			GrabLoop_ProvidedByInstantCamera);

		chase_controller_->start();
		start_chase_feed_thread();
		start_operator_thread();

		phase_ = ExperimentPhase::Streaming;
		readiness_.ferret_identified = ComponentStatus::Calibrating;
		readiness_.prey_identified = ComponentStatus::Calibrating;
		log_info("experiment",
			"Streaming — warmup 30s, then s=start e=end r=reset chase trial");
		return true;
	} catch (const GenericException& e) {
		log_error("experiment", std::string("Pylon error: ") + e.GetDescription());
		phase_ = ExperimentPhase::Error;
		return false;
	}
}

void ExperimentStateManager::update_identity_status(const TrackingFrame& frame) {
	if (frame.ferret.state.valid
		&& frame.quality.ferret_confidence >= kIdentityConfidenceMin) {
		++ferret_stable_frames_;
	} else {
		ferret_stable_frames_ = 0;
	}
	if (frame.prey.state.valid
		&& frame.quality.prey_confidence >= kIdentityConfidenceMin) {
		++prey_stable_frames_;
	} else {
		prey_stable_frames_ = 0;
	}
	if (ferret_stable_frames_ >= static_cast<uint32_t>(kIdentityStableFrames)) {
		readiness_.ferret_identified = ComponentStatus::Ready;
	}
	if (prey_stable_frames_ >= static_cast<uint32_t>(kIdentityStableFrames)) {
		readiness_.prey_identified = ComponentStatus::Ready;
	}
}

void ExperimentStateManager::update_experiment_phase() {
	if (trial_fsm_ && trial_fsm_->phase() == TrialPhase::Running) {
		phase_ = ExperimentPhase::ChaseSession;
	} else if (phase_ == ExperimentPhase::ChaseSession) {
		phase_ = ExperimentPhase::Streaming;
	}
}

void ExperimentStateManager::log_status_summary() const {
	std::ostringstream oss;
	oss << "phase=" << experiment_phase_name(phase_)
	    << " camera=" << component_status_name(readiness_.camera)
	    << " motor=" << component_status_name(readiness_.motor)
	    << " ferret=" << component_status_name(readiness_.ferret_identified)
	    << " prey=" << component_status_name(readiness_.prey_identified);
	if (trial_fsm_) {
		oss << " trial=" << trial_phase_name(trial_fsm_->phase());
	}
	log_info("experiment", oss.str());
}

void ExperimentStateManager::on_operator_key(char key) {
	const int64_t ts = host_time_ns();
	if (key == 's') {
		if (readiness_.motor != ComponentStatus::Ready) {
			log_info("experiment", "Cannot start — motor not ready");
			return;
		}
		if (trial_fsm_->start_trial()) {
			log_info("experiment", "Trial started (chase running)");
			recorder_->log_event("trial_start", ts, trial_fsm_->phase());
		}
	} else if (key == 'e') {
		if (trial_fsm_->end_trial()) {
			log_info("experiment", "Trial ended");
			recorder_->log_event("trial_end", ts, trial_fsm_->phase());
		}
	} else if (key == 'r') {
		if (trial_fsm_->reset_trial()) {
			log_info("experiment", "Trial reset — armed for next start");
			recorder_->log_event("trial_reset", ts, trial_fsm_->phase());
		}
	}
}

void ExperimentStateManager::start_operator_thread() {
	operator_thread_ = std::thread([this]() {
		while (chase_feed_running_.load()) {
			char key = 0;
			if (!(std::cin >> key)) {
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
				continue;
			}
			on_operator_key(key);
		}
	});
}

void ExperimentStateManager::start_chase_feed_thread() {
	chase_feed_running_.store(true);
	chase_feed_thread_ = std::thread(&ExperimentStateManager::chase_feed_loop, this);
}

void ExperimentStateManager::chase_feed_loop() {
	while (chase_feed_running_.load()) {
		TrackingFrame frame;
		if (tracking_ && tracking_->get_tracking_frame(frame)) {
			if (chase_controller_) {
				chase_controller_->submit_frame(frame);
			}
			update_identity_status(frame);
			update_experiment_phase();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(kMainLoopSleepMs));
	}
}

void ExperimentStateManager::main_loop(std::atomic<bool>& running) {
	auto last_status = std::chrono::steady_clock::now();
	while (running.load()) {
		const auto now = std::chrono::steady_clock::now();
		if (now - last_status >= std::chrono::milliseconds(kStatusLogIntervalMs)) {
			log_status_summary();
			last_status = now;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(kMainLoopSleepMs));
	}
}

void ExperimentStateManager::shutdown() {
	if (shutdown_done_) {
		return;
	}
	shutdown_done_ = true;
	chase_feed_running_.store(false);
	if (chase_controller_) {
		chase_controller_->stop();
	}
	if (prey_motor_) {
		prey_motor_->estop();
	}
	if (trap_motor_) {
		trap_motor_->estop();
	}
	if (camera_) {
		try {
			camera_->StopGrabbing();
		} catch (...) {}
	}
	if (display_) {
		display_->stop();
	}
	if (operator_thread_.joinable()) {
		operator_thread_.join();
	}
	if (chase_feed_thread_.joinable()) {
		chase_feed_thread_.join();
	}
	tracking_.reset();
	camera_.reset();
	phase_ = ExperimentPhase::Ended;
	log_info("experiment", "Experiment ended");
}

int ExperimentStateManager::run(std::atomic<bool>& running) {
	PylonInitialize();
	int exit_code = 0;
	try {
		if (!phase_setup() || !verify_setup_artifacts() || !phase_configuring()) {
			exit_code = 1;
		} else {
			if (opts_.enable_display && tracking_) {
				display_ = std::make_unique<DisplayThread>();
				display_->start(tracking_.get(), &running);
			}
			main_loop(running);
		}
	} catch (const std::exception& e) {
		log_error("experiment", std::string("Fatal: ") + e.what());
		phase_ = ExperimentPhase::Error;
		exit_code = 1;
	}
	shutdown();
	PylonTerminate();
	return exit_code;
}

