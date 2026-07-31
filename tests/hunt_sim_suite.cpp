// Motor-only hunt simulation: ~2 minutes of varied chain moves (no camera).
// Cycles short/fast bursts, longer/slower flees, reverses, pauses, and
// accel extremes, then scores distance + speed accuracy vs the runtime plan.
//
// Usage:
//   test_hunt_sim [--config <arena_experiment.json>] [--duration-s 120]
//                 [--max-accel 50] [--csv hunt_sim.csv] [--verbose] [--dry-run]
//
// Example:
//   ./bin/test_hunt_sim --config ../config/arena_experiment.json --csv out.csv

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "calibrate/setup_util.h"
#include "experiment/arena_config.h"
#include "log/logger.h"
#include "motor/motion_planner.h"
#include "motor/motor_config.h"
#include "motor/prey_motor.h"

namespace {

MotionPlanner* g_planner = nullptr;
std::atomic<bool> g_stop{false};

void signal_handler(int) {
	g_stop.store(true);
	if (g_planner) {
		g_planner->cancel();
	}
}

struct Args {
	std::string config_path;
	std::string csv_path;
	float duration_s = 120.0f;
	float max_accel_mps2 = 50.0f; // default matches rig-proven fast ramp
	bool verbose = false;
	bool dry_run = false;
};

// One scripted flee/creep-like segment in the hunt playlist.
struct HuntSegment {
	const char* label;
	float distance_mm;   // signed chain travel
	float duration_s;    // planned move window
	float accel_mps2;    // 0 → use CLI --max-accel
	float pause_after_s; // dwell before next segment
};

// Per-move encoder vs plan accuracy + command timing (pauses omitted from aggregates).
struct MoveMetrics {
	std::string label;
	bool ok = false;
	float req_mm = 0.0f;
	float plan_mm = 0.0f;
	float actual_mm = 0.0f;
	float plan_duration_s = 0.0f;
	float actual_duration_s = 0.0f;
	float plan_avg_mps = 0.0f;
	float actual_avg_mps = 0.0f;
	float plan_peak_turns_s = 0.0f;
	float actual_peak_turns_s = 0.0f;
	float actual_mean_turns_s = 0.0f;
	// Within-move: intervals between consecutive Set_Input_Vel ticks (~20 ms).
	int cmd_count = 0;
	float cmd_dt_mean_ms = 0.0f;
	float cmd_dt_min_ms = 0.0f;
	float cmd_dt_max_ms = 0.0f;
	float cmd_dt_stdev_ms = 0.0f;
	// Between moves: idle gap after previous move (includes planned pause).
	float gap_from_prev_ms = -1.0f;     // -1 = first move
	float plan_pause_before_ms = 0.0f;  // previous segment pause_after
	float start_to_start_ms = -1.0f;    // previous move start → this start
};

// Tracks wall times so consecutive-command gaps span segment boundaries.
struct TimingState {
	std::chrono::steady_clock::time_point last_move_end{};
	std::chrono::steady_clock::time_point last_move_start{};
	float last_pause_after_s = 0.0f;
	bool have_prev_move = false;
};

float ms_between(std::chrono::steady_clock::time_point a,
	std::chrono::steady_clock::time_point b)
{
	return std::chrono::duration<float, std::milli>(b - a).count();
}

void accumulate_dt(float dt_ms, float& sum, float& sum_sq, float& mn, float& mx,
	int& n)
{
	if (n == 0) {
		mn = mx = dt_ms;
	} else {
		mn = std::min(mn, dt_ms);
		mx = std::max(mx, dt_ms);
	}
	sum += dt_ms;
	sum_sq += dt_ms * dt_ms;
	++n;
}

void finalize_dt_stats(int n, float sum, float sum_sq, float mn, float mx,
	MoveMetrics& m)
{
	if (n <= 0) {
		return;
	}
	m.cmd_dt_mean_ms = sum / static_cast<float>(n);
	m.cmd_dt_min_ms = mn;
	m.cmd_dt_max_ms = mx;
	if (n > 1) {
		const float var = (sum_sq / static_cast<float>(n))
			- (m.cmd_dt_mean_ms * m.cmd_dt_mean_ms);
		m.cmd_dt_stdev_ms = std::sqrt(std::max(0.0f, var));
	}
}

void print_usage() {
	std::cerr <<
		"Usage: test_hunt_sim [--config <arena_experiment.json>]\n"
		"                     [--duration-s 120] [--max-accel 50]\n"
		"                     [--csv <path>] [--verbose] [--dry-run]\n"
		"\n"
		"  Runs a looping playlist of varied chain moves for ~duration-s\n"
		"  (default 120). Reports distance/speed accuracy and command timing\n"
		"  (inter-tick dt + gaps between consecutive moves). Ctrl-C aborts.\n";
}

bool parse_args(int argc, char** argv, Args& args) {
	try {
		for (int i = 1; i < argc; ++i) {
			if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
				args.config_path = argv[++i];
			} else if (std::strcmp(argv[i], "--duration-s") == 0 && i + 1 < argc) {
				args.duration_s = std::stof(argv[++i]);
			} else if (std::strcmp(argv[i], "--max-accel") == 0 && i + 1 < argc) {
				args.max_accel_mps2 = std::stof(argv[++i]);
			} else if (std::strcmp(argv[i], "--csv") == 0 && i + 1 < argc) {
				args.csv_path = argv[++i];
			} else if (std::strcmp(argv[i], "--verbose") == 0) {
				args.verbose = true;
			} else if (std::strcmp(argv[i], "--dry-run") == 0) {
				args.dry_run = true;
			} else if (std::strcmp(argv[i], "--help") == 0) {
				return false;
			} else {
				std::cerr << "Unknown argument: " << argv[i] << '\n';
				return false;
			}
		}
	} catch (const std::exception& e) {
		std::cerr << "Invalid numeric argument: " << e.what() << '\n';
		return false;
	}
	return args.duration_s > 0.0f && args.max_accel_mps2 > 0.0f;
}

// Fixed playlist — loops until wall clock hits --duration-s.
// Distances/times chosen so many peaks clear ~1.5 turns/s @ 660 mm/turn;
// a few slow segments intentionally stress the min-viable floor.
std::vector<HuntSegment> make_playlist() {
	return {
		{"short_fast_fwd",      180.0f,  0.25f, 50.0f, 0.15f},
		{"short_fast_rev",     -160.0f,  0.22f, 50.0f, 0.15f},
		{"medium_flee_fwd",     400.0f,  0.55f, 20.0f, 0.25f},
		{"pause_listen",          0.0f,  0.00f,  0.0f, 0.40f},
		{"long_slower_fwd",     700.0f,  1.20f,  8.0f, 0.30f},
		{"burst_double_a",      120.0f,  0.18f, 50.0f, 0.05f},
		{"burst_double_b",      140.0f,  0.20f, 50.0f, 0.20f},
		{"medium_flee_rev",    -450.0f,  0.65f, 15.0f, 0.25f},
		{"creep_floor_stress",   80.0f,  0.60f,  5.0f, 0.20f}, // low peak → floor
		{"long_slow_rev",      -800.0f,  1.60f,  5.0f, 0.35f},
		{"jab_fwd",             100.0f,  0.15f, 50.0f, 0.10f},
		{"jab_rev",            -100.0f,  0.15f, 50.0f, 0.10f},
		{"wide_flee_fwd",       550.0f,  0.80f, 25.0f, 0.30f},
		{"soft_accel_fwd",      300.0f,  0.70f,  2.0f, 0.25f},
		{"hard_accel_rev",     -250.0f,  0.35f, 50.0f, 0.20f},
		{"micro_pause",           0.0f,  0.00f,  0.0f, 0.50f},
		{"sprint_fwd",          220.0f,  0.28f, 50.0f, 0.15f},
		{"drift_fwd",           500.0f,  1.00f, 10.0f, 0.30f},
	};
}

float playlist_cycle_seconds(const std::vector<HuntSegment>& playlist) {
	float total = 0.0f;
	for (const HuntSegment& s : playlist) {
		total += s.duration_s + s.pause_after_s;
	}
	return total;
}

bool connect_motor(const ArenaExperimentConfig& cfg, PreyMotor& motor) {
	if (!can_interface_up(cfg.motor.can_interface)) {
		log_error("hunt_sim", "CAN interface " + cfg.motor.can_interface
			+ " is not up");
		return false;
	}
	if (!motor.connect()) {
		log_error("hunt_sim", "Prey motor connect failed");
		return false;
	}
	if (!motor.status().heartbeat_ok) {
		log_error("hunt_sim", "Prey motor heartbeat failed");
		return false;
	}
	if (!motor.enter_velocity_mode()) {
		log_error("hunt_sim", "Failed to enter closed-loop velocity mode");
		return false;
	}
	return true;
}

void sleep_interruptible_ms(int pause_ms) {
	for (int t = 0; t < pause_ms && !g_stop.load(); t += 20) {
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}
}

// Execute plan at ~50 Hz; sample encoder vel + inter-command tick intervals.
bool execute_measured(PreyMotor& motor, MotionPlanner& planner,
	const ChainMovePlan& plan, MoveMetrics& m)
{
	const float pos0 = motor.read_position_turns();
	const auto t0 = std::chrono::steady_clock::now();
	if (!planner.start_plan(plan)) {
		return false;
	}

	float peak_abs_turns_s = 0.0f;
	float sum_abs_turns_s = 0.0f;
	int n_vel = 0;
	float dt_sum = 0.0f;
	float dt_sum_sq = 0.0f;
	float dt_min = 0.0f;
	float dt_max = 0.0f;
	int n_dt = 0;
	bool have_prev_cmd = false;
	std::chrono::steady_clock::time_point prev_cmd{};
	bool ok = false;
	while (!g_stop.load()) {
		// Why: stamp before tick — interval is consecutive command issue times.
		const auto cmd_t = std::chrono::steady_clock::now();
		if (have_prev_cmd) {
			accumulate_dt(ms_between(prev_cmd, cmd_t), dt_sum, dt_sum_sq,
				dt_min, dt_max, n_dt);
		}
		prev_cmd = cmd_t;
		have_prev_cmd = true;
		++m.cmd_count;

		const MoveTick tick = planner.tick(motor);
		// Non-blocking encoder peek — never stall the Set_Input_Vel cadence.
		float vel_sample = 0.0f;
		if (motor.try_sample_velocity_turns_s(vel_sample, /*timeout_ms=*/0)) {
			const float vel = std::fabs(vel_sample);
			peak_abs_turns_s = std::max(peak_abs_turns_s, vel);
			sum_abs_turns_s += vel;
			++n_vel;
		}
		if (tick == MoveTick::Done) {
			ok = true;
			break;
		}
		if (tick == MoveTick::Cancelled || tick == MoveTick::Idle) {
			ok = false;
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(20));
	}

	finalize_dt_stats(n_dt, dt_sum, dt_sum_sq, dt_min, dt_max, m);
	m.actual_duration_s = std::chrono::duration<float>(
		std::chrono::steady_clock::now() - t0).count();
	m.actual_mm = motor.turns_to_chain_mm(motor.read_position_turns() - pos0);
	m.actual_peak_turns_s = peak_abs_turns_s;
	m.actual_mean_turns_s = (n_vel > 0) ? (sum_abs_turns_s / n_vel) : 0.0f;
	if (m.actual_duration_s > 1e-3f) {
		m.actual_avg_mps = (m.actual_mm / 1000.0f) / m.actual_duration_s;
	}
	m.ok = ok && !g_stop.load();
	return m.ok;
}

void fill_plan_targets(const HuntSegment& seg, const ChainMovePlan& plan,
	MoveMetrics& m)
{
	m.label = seg.label;
	m.req_mm = seg.distance_mm;
	m.plan_mm = plan.expected_distance_mm;
	m.plan_duration_s = plan.duration_s;
	m.plan_avg_mps = plan.avg_speed_mps;
	// Why: avg_speed_mps is request/T_request; after floor, use plan travel/time.
	if (plan.duration_s > 1e-3f) {
		m.plan_avg_mps = (plan.expected_distance_mm / 1000.0f) / plan.duration_s;
	}
	m.plan_peak_turns_s = std::fabs(plan.peak_turns_s);
}

void print_move_line(const MoveMetrics& m) {
	const float req_err = m.actual_mm - m.req_mm;
	const float plan_err = m.actual_mm - m.plan_mm;
	const float peak_err = m.actual_peak_turns_s - m.plan_peak_turns_s;
	const float peak_pct = (m.plan_peak_turns_s > 1e-3f)
		? (100.0f * peak_err / m.plan_peak_turns_s) : 0.0f;
	const float avg_err_mps = m.actual_avg_mps - m.plan_avg_mps;
	std::cout << std::fixed << std::setprecision(1)
		<< "    " << (m.ok ? "ok" : "FAIL")
		<< "  dist req=" << m.req_mm << " plan=" << m.plan_mm
		<< " act=" << m.actual_mm
		<< "  err_req=" << req_err << " err_plan=" << plan_err << " mm\n"
		<< "    speed avg plan=" << (m.plan_avg_mps * 1000.0f)
		<< " act=" << (m.actual_avg_mps * 1000.0f)
		<< " err=" << (avg_err_mps * 1000.0f) << " mm/s\n"
		<< "    peak turns/s plan=" << m.plan_peak_turns_s
		<< " act=" << m.actual_peak_turns_s
		<< " err=" << peak_err << " (" << peak_pct << "%)"
		<< "  T plan=" << m.plan_duration_s
		<< " act=" << m.actual_duration_s << " s\n"
		<< "    cmd dt mean/min/max/stdev="
		<< m.cmd_dt_mean_ms << "/" << m.cmd_dt_min_ms << "/"
		<< m.cmd_dt_max_ms << "/" << m.cmd_dt_stdev_ms << " ms"
		<< "  n=" << m.cmd_count;
	if (m.gap_from_prev_ms >= 0.0f) {
		std::cout << "  gap_prev=" << m.gap_from_prev_ms
			<< " ms (plan pause " << m.plan_pause_before_ms << ")"
			<< "  startΔ=" << m.start_to_start_ms << " ms";
	}
	std::cout << "\n";
}

void stamp_inter_move_gaps(TimingState& timing, MoveMetrics& m,
	std::chrono::steady_clock::time_point move_start)
{
	if (!timing.have_prev_move) {
		return;
	}
	// Idle gap = previous move end → this move start (should ≈ planned pause).
	m.gap_from_prev_ms = ms_between(timing.last_move_end, move_start);
	m.plan_pause_before_ms = timing.last_pause_after_s * 1000.0f;
	m.start_to_start_ms = ms_between(timing.last_move_start, move_start);
}

bool run_segment(PreyMotor& motor, MotionPlanner& planner, const HuntSegment& seg,
	float default_accel, int index, float elapsed_s, TimingState& timing,
	std::vector<MoveMetrics>& out)
{
	std::cout << "[" << static_cast<int>(elapsed_s) << "s] #" << index << " "
		<< seg.label << "  dist=" << seg.distance_mm << " mm  T="
		<< seg.duration_s << " s\n";

	if (std::fabs(seg.distance_mm) < 1e-3f) {
		// Pure dwell still advances the inter-move timeline for the next command.
		sleep_interruptible_ms(static_cast<int>(seg.pause_after_s * 1000.0f));
		if (timing.have_prev_move) {
			timing.last_pause_after_s += seg.pause_after_s;
		}
		return !g_stop.load();
	}

	const float accel = (seg.accel_mps2 > 0.0f) ? seg.accel_mps2 : default_accel;
	const int duration_ms = static_cast<int>(std::lround(seg.duration_s * 1000.0f));
	const ChainMovePlan plan = MotionPlanner::runtime_plan_distance_mm_in_time(
		motor, seg.distance_mm, duration_ms, accel);
	if (!plan.feasible) {
		std::cout << "    SKIP — plan not feasible\n";
		return false;
	}

	MoveMetrics m;
	fill_plan_targets(seg, plan, m);
	const auto move_start = std::chrono::steady_clock::now();
	stamp_inter_move_gaps(timing, m, move_start);
	const bool ok = execute_measured(motor, planner, plan, m);
	const auto move_end = std::chrono::steady_clock::now();
	print_move_line(m);
	out.push_back(m);

	timing.last_move_start = move_start;
	timing.last_move_end = move_end;
	timing.last_pause_after_s = seg.pause_after_s;
	timing.have_prev_move = true;

	sleep_interruptible_ms(static_cast<int>(seg.pause_after_s * 1000.0f));
	return ok && !g_stop.load();
}

float mae(const std::vector<float>& err) {
	if (err.empty()) {
		return 0.0f;
	}
	float s = 0.0f;
	for (float e : err) {
		s += std::fabs(e);
	}
	return s / static_cast<float>(err.size());
}

float rmse(const std::vector<float>& err) {
	if (err.empty()) {
		return 0.0f;
	}
	float s = 0.0f;
	for (float e : err) {
		s += e * e;
	}
	return std::sqrt(s / static_cast<float>(err.size()));
}

int count_within_pct(const std::vector<float>& abs_pct, float limit) {
	int n = 0;
	for (float p : abs_pct) {
		if (p <= limit) {
			++n;
		}
	}
	return n;
}

// Why: keep max-|err| helper tiny so summary stays readable.
float max_abs(const std::vector<float>& err) {
	float m = 0.0f;
	for (float e : err) {
		m = std::max(m, std::fabs(e));
	}
	return m;
}

void print_score_block(const char* title, const std::vector<float>& err,
	const std::vector<float>& abs_pct, const char* unit)
{
	const int n = static_cast<int>(err.size());
	std::cout << title << "\n"
		<< "  MAE=" << mae(err) << " " << unit
		<< "  RMSE=" << rmse(err) << " " << unit
		<< "  max|err|=" << max_abs(err) << " " << unit << "\n"
		<< "  mean |err|%=" << mae(abs_pct) << "%"
		<< "  within 5%=" << count_within_pct(abs_pct, 5.0f) << "/" << n
		<< "  within 10%=" << count_within_pct(abs_pct, 10.0f) << "/" << n
		<< "\n";
}

void print_timing_summary(const std::vector<MoveMetrics>& moves) {
	std::vector<float> cmd_means;
	std::vector<float> cmd_maxes;
	std::vector<float> gap_err; // actual idle gap − planned pause
	std::vector<float> gap_pct;
	float cmd_min_all = 1e9f;
	float cmd_max_all = 0.0f;
	int cmd_total = 0;
	for (const MoveMetrics& m : moves) {
		if (!m.ok) {
			continue;
		}
		cmd_means.push_back(m.cmd_dt_mean_ms);
		cmd_maxes.push_back(m.cmd_dt_max_ms);
		cmd_min_all = std::min(cmd_min_all, m.cmd_dt_min_ms);
		cmd_max_all = std::max(cmd_max_all, m.cmd_dt_max_ms);
		cmd_total += m.cmd_count;
		if (m.gap_from_prev_ms < 0.0f) {
			continue;
		}
		const float err = m.gap_from_prev_ms - m.plan_pause_before_ms;
		gap_err.push_back(err);
		gap_pct.push_back((m.plan_pause_before_ms > 1e-3f)
			? 100.0f * std::fabs(err) / m.plan_pause_before_ms : 0.0f);
	}
	std::cout << "Command timing (target tick ~20 ms):\n"
		<< "  ticks=" << cmd_total
		<< "  mean-of-means=" << mae(cmd_means) << " ms"
		<< "  min=" << (cmd_means.empty() ? 0.0f : cmd_min_all) << " ms"
		<< "  max=" << cmd_max_all << " ms"
		<< "  worst-move-max=" << (cmd_maxes.empty() ? 0.0f : max_abs(cmd_maxes))
		<< " ms\n";
	if (!gap_err.empty()) {
		print_score_block("Idle gap vs planned pause:", gap_err, gap_pct, "ms");
	}
}

void print_full_summary(const std::vector<MoveMetrics>& moves) {
	std::vector<float> req_err, req_pct, plan_err, plan_pct;
	std::vector<float> avg_err, avg_pct, peak_err, peak_pct;
	int ok_n = 0;
	for (const MoveMetrics& m : moves) {
		if (!m.ok) {
			continue;
		}
		++ok_n;
		req_err.push_back(m.actual_mm - m.req_mm);
		req_pct.push_back((std::fabs(m.req_mm) > 1e-3f)
			? 100.0f * std::fabs(m.actual_mm - m.req_mm) / std::fabs(m.req_mm)
			: 0.0f);
		plan_err.push_back(m.actual_mm - m.plan_mm);
		plan_pct.push_back((std::fabs(m.plan_mm) > 1e-3f)
			? 100.0f * std::fabs(m.actual_mm - m.plan_mm) / std::fabs(m.plan_mm)
			: 0.0f);
		avg_err.push_back((m.actual_avg_mps - m.plan_avg_mps) * 1000.0f);
		avg_pct.push_back((std::fabs(m.plan_avg_mps) > 1e-6f)
			? 100.0f * std::fabs(m.actual_avg_mps - m.plan_avg_mps)
				/ std::fabs(m.plan_avg_mps)
			: 0.0f);
		peak_err.push_back(m.actual_peak_turns_s - m.plan_peak_turns_s);
		peak_pct.push_back((m.plan_peak_turns_s > 1e-3f)
			? 100.0f * std::fabs(m.actual_peak_turns_s - m.plan_peak_turns_s)
				/ m.plan_peak_turns_s
			: 0.0f);
	}

	std::cout << "\n=== Motor performance (" << ok_n << "/" << moves.size()
		<< " ok moves) ===\n" << std::fixed << std::setprecision(2);
	if (ok_n == 0) {
		std::cout << "No completed moves to score.\n";
		return;
	}
	// Request = playlist target; plan = runtime profile (may lift min-viable floor).
	print_score_block("Distance vs request:", req_err, req_pct, "mm");
	print_score_block("Distance vs plan:", plan_err, plan_pct, "mm");
	print_score_block("Mean speed vs plan:", avg_err, avg_pct, "mm/s");
	print_score_block("Peak speed vs plan:", peak_err, peak_pct, "turns/s");
	print_timing_summary(moves);
}

bool write_csv(const std::string& path, const std::vector<MoveMetrics>& moves) {
	std::ofstream out(path);
	if (!out) {
		log_error("hunt_sim", "Failed to write CSV: " + path);
		return false;
	}
	out << "label,ok,req_mm,plan_mm,actual_mm,dist_err_mm,dist_err_pct,"
		<< "plan_duration_s,actual_duration_s,"
		<< "plan_avg_mmps,actual_avg_mmps,avg_err_mmps,"
		<< "plan_peak_turns_s,actual_peak_turns_s,peak_err_turns_s,"
		<< "actual_mean_turns_s,"
		<< "cmd_count,cmd_dt_mean_ms,cmd_dt_min_ms,cmd_dt_max_ms,cmd_dt_stdev_ms,"
		<< "gap_from_prev_ms,plan_pause_before_ms,start_to_start_ms\n";
	out << std::fixed << std::setprecision(4);
	for (const MoveMetrics& m : moves) {
		const float derr = m.actual_mm - m.plan_mm;
		const float dpct = (std::fabs(m.plan_mm) > 1e-3f)
			? 100.0f * derr / m.plan_mm : 0.0f;
		out << m.label << "," << (m.ok ? 1 : 0) << ","
			<< m.req_mm << "," << m.plan_mm << "," << m.actual_mm << ","
			<< derr << "," << dpct << ","
			<< m.plan_duration_s << "," << m.actual_duration_s << ","
			<< (m.plan_avg_mps * 1000.0f) << ","
			<< (m.actual_avg_mps * 1000.0f) << ","
			<< ((m.actual_avg_mps - m.plan_avg_mps) * 1000.0f) << ","
			<< m.plan_peak_turns_s << "," << m.actual_peak_turns_s << ","
			<< (m.actual_peak_turns_s - m.plan_peak_turns_s) << ","
			<< m.actual_mean_turns_s << ","
			<< m.cmd_count << "," << m.cmd_dt_mean_ms << ","
			<< m.cmd_dt_min_ms << "," << m.cmd_dt_max_ms << ","
			<< m.cmd_dt_stdev_ms << ","
			<< m.gap_from_prev_ms << "," << m.plan_pause_before_ms << ","
			<< m.start_to_start_ms << "\n";
	}
	std::cout << "Wrote " << path << " (" << moves.size() << " rows)\n";
	return true;
}

} // namespace

int main(int argc, char** argv) {
	Args args;
	if (!parse_args(argc, argv, args)) {
		print_usage();
		return 1;
	}
	Logger::instance().set_level(args.verbose ? LogLevel::Debug : LogLevel::Info);

	const std::vector<HuntSegment> playlist = make_playlist();
	const float cycle_s = playlist_cycle_seconds(playlist);
	std::cout << "Hunt sim playlist: " << playlist.size() << " segments, ~"
		<< cycle_s << " s/cycle, run for " << args.duration_s << " s\n";

	if (args.dry_run) {
		int i = 0;
		for (const HuntSegment& s : playlist) {
			std::cout << "  " << (++i) << ". " << s.label
				<< "  " << s.distance_mm << " mm / " << s.duration_s
				<< " s  accel=" << s.accel_mps2
				<< "  pause=" << s.pause_after_s << " s\n";
		}
		return 0;
	}

	const std::string config_path = resolve_arena_config_path(argv[0], args.config_path);
	if (config_path.empty()) {
		log_error("hunt_sim", "No arena config — pass --config <path>");
		return 1;
	}
	ArenaExperimentConfig cfg;
	if (!load_arena_experiment_config(config_path, cfg)) {
		return 1;
	}
	if (cfg.motor.chain_mm_per_motor_turn <= 0.0f) {
		log_error("hunt_sim", "motor.chain_mm_per_motor_turn missing");
		return 1;
	}

	PreyMotor motor(prey_motor_from_config(cfg.motor));
	if (!connect_motor(cfg, motor)) {
		return 1;
	}

	MotionPlanner planner;
	g_planner = &planner;
	std::signal(SIGINT, signal_handler);
	std::signal(SIGTERM, signal_handler);

	const auto t0 = std::chrono::steady_clock::now();
	const float pos_start = motor.read_position_turns();
	int segment_index = 0;
	int completed = 0;
	int failed = 0;
	std::vector<MoveMetrics> metrics;
	TimingState timing;

	std::cout << "Starting hunt sim — Ctrl-C to abort.\n";
	while (!g_stop.load()) {
		const float elapsed_s = std::chrono::duration<float>(
			std::chrono::steady_clock::now() - t0).count();
		if (elapsed_s >= args.duration_s) {
			break;
		}
		const HuntSegment& seg = playlist[
			static_cast<size_t>(segment_index) % playlist.size()];
		++segment_index;
		if (!run_segment(motor, planner, seg, args.max_accel_mps2,
				segment_index, elapsed_s, timing, metrics)) {
			++failed;
			if (g_stop.load()) {
				break;
			}
		} else {
			++completed;
		}
	}

	motor.stop();
	const float net_mm = motor.turns_to_chain_mm(
		motor.read_position_turns() - pos_start);
	const float elapsed_s = std::chrono::duration<float>(
		std::chrono::steady_clock::now() - t0).count();
	std::cout << "\nHunt sim finished in " << elapsed_s << " s\n"
		<< "Segments completed=" << completed << " failed/cancelled=" << failed
		<< "\nNet chain travel=" << net_mm << " mm\n";

	print_full_summary(metrics);
	if (!args.csv_path.empty()) {
		write_csv(args.csv_path, metrics);
	}

	return (g_stop.load() || failed > 0) ? 1 : 0;
}
