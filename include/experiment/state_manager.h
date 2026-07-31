#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include "experiment/arena_config.h"
#include "vision/tracking_frame.h"

namespace Pylon {
class CBaslerUniversalInstantCamera;
}

enum class ComponentStatus : uint8_t {
	NotStarted,
	Calibrating,
	Ready,
	Error
};

enum class ExperimentPhase : uint8_t {
	Idle,
	Setup,
	Configuring,
	Streaming,
	ChaseSession,
	Ended,
	Error
};

struct ExperimentReadiness {
	ComponentStatus camera = ComponentStatus::NotStarted;
	ComponentStatus motor = ComponentStatus::NotStarted;
	ComponentStatus shuttle = ComponentStatus::NotStarted;
	ComponentStatus ferret_identified = ComponentStatus::NotStarted;
	ComponentStatus prey_identified = ComponentStatus::NotStarted;
};

const char* component_status_name(ComponentStatus status);
const char* experiment_phase_name(ExperimentPhase phase);

struct ExperimentOptions {
	const char* argv0 = nullptr;
	std::string arena_config_path;
	std::string session_dir = "sessions";
	std::string camera_config_path;
	std::string calib_path;
	bool disable_calib = false;
	bool enable_display = false;
	bool verbose = false;
	std::string log_file;
};

// Per-session experiment orchestrator (run subcommand only).
class ExperimentStateManager {
public:
	explicit ExperimentStateManager(ExperimentOptions opts);

	~ExperimentStateManager();

	int run(std::atomic<bool>& running);

	ExperimentPhase phase() const { return phase_; }
	ExperimentReadiness readiness() const { return readiness_; }

private:
	static constexpr int kIdentityStableFrames = 30;
	static constexpr float kIdentityConfidenceMin = 0.5f;
	static constexpr float kGsdMmPx = 1.035f;
	static constexpr float kFps = 200.0f;
	static constexpr int kWarmupFrames = static_cast<int>(kFps * 30);

	ExperimentOptions opts_;
	ArenaExperimentConfig cfg_;

	std::unique_ptr<class PreyMotor> prey_motor_;
	std::unique_ptr<class ShuttleMotor> shuttle_motor_;
	std::unique_ptr<class MotionPlanner> motion_planner_;
	std::unique_ptr<class ChaseController> chase_controller_;
	std::unique_ptr<class SessionRecorder> recorder_;
	std::unique_ptr<class TrialStateMachine> trial_fsm_;
	std::unique_ptr<class CameraTrackingService> tracking_;
	std::unique_ptr<Pylon::CBaslerUniversalInstantCamera> camera_;
	std::unique_ptr<class DisplayThread> display_;

	ExperimentPhase phase_ = ExperimentPhase::Idle;
	ExperimentReadiness readiness_{};
	uint32_t ferret_stable_frames_ = 0;
	uint32_t prey_stable_frames_ = 0;

	std::thread operator_thread_;
	std::thread chase_feed_thread_;
	std::atomic<bool> chase_feed_running_{false};
	bool shutdown_done_ = false;

	bool phase_setup();
	bool phase_configuring();
	void main_loop(std::atomic<bool>& running);
	void shutdown();

	bool verify_setup_artifacts();
	bool connect_runtime_hardware();

	void start_operator_thread();
	void start_chase_feed_thread();
	void chase_feed_loop();
	void update_identity_status(const TrackingFrame& frame);
	void update_experiment_phase();
	void log_status_summary() const;
	void on_operator_key(char key);
};
