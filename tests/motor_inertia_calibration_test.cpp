// Chain motor inertia/friction calibration — motor-only, no camera involved.
//
// Two-trial step-response sweep of the prey chain motor over a turns/s range,
// then a physical system-ID regression fit against every sample from both
// trials. See tests/README.md for the full protocol writeup and how the
// result feeds PreyMotor's Set_Input_Vel torque feed-forward.
//
// IMPORTANT: the prey chain must be a closed loop (no physical end) — this
// test spins continuously for several minutes and does not track position.
//
// Usage:
//   test_motor_inertia_calibration [--config <arena_experiment.json>]
//       [--rps-min 1.0] [--rps-max 6.0] [--rps-step 0.2] [--hold-s 2.0]
//       [--settle-tol 0.05] [--settle-hold-s 0.2] [--sample-hz 100]
//       [--torque-constant 0.0827] [--max-step-wait-s 10] [--output <dir>]
//       [--write-config] [--verbose]

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include "calibrate/setup_util.h"
#include "experiment/arena_config.h"
#include "log/logger.h"
#include "motor/motor_config.h"
#include "motor/prey_motor.h"

namespace {

constexpr float kTwoPi = 6.283185307f;

PreyMotor* g_motor = nullptr;
std::atomic<bool> g_stop{false};

void signal_handler(int) {
	g_stop.store(true);
	if (g_motor) {
		g_motor->stop();
	}
}

struct Args {
	std::string config_path;
	std::string output_dir = "tests/output";
	float rps_min = 1.0f;
	float rps_max = 6.0f;
	float rps_step = 0.2f;
	float hold_s = 2.0f;
	float settle_tol = 0.05f;
	float settle_hold_s = 0.2f;
	float sample_hz = 100.0f;
	float torque_constant = 0.0827f;
	float max_step_wait_s = 10.0f;
	// If still unsettled right at max_step_wait_s but already this close to
	// target, grant one grace_s extension instead of giving up — a slow
	// final approach is still real dynamics, and settling faster on the
	// second attempt vs. not settling at all is itself informative.
	float near_tol = 0.1f;
	float grace_s = 1.0f;
	bool write_config = false;
	bool verbose = false;
};

void print_usage() {
	std::cerr <<
		"Usage: test_motor_inertia_calibration [--config <arena_experiment.json>]\n"
		"    [--rps-min 1.0] [--rps-max 6.0] [--rps-step 0.2] [--hold-s 2.0]\n"
		"    [--settle-tol 0.05] [--settle-hold-s 0.2] [--sample-hz 100]\n"
		"    [--torque-constant 0.0827] [--max-step-wait-s 10] [--near-tol 0.1]\n"
		"    [--grace-s 1.0] [--output <dir>] [--write-config] [--verbose]\n"
		"\n"
		"  Two-trial step-response sweep of the prey chain motor from --rps-min\n"
		"  to --rps-max in --rps-step increments (turns/s), then fits a physical\n"
		"  torque = J*alpha + B*omega + tau_c*sign(omega) model to every sample\n"
		"  from both trials. Prints the calibrated chain_inertia_kg_m2 /\n"
		"  chain_viscous_friction_nm_s_per_rad / chain_static_friction_nm.\n"
		"\n"
		"  The chain MUST be a closed loop (no physical end) — this test spins\n"
		"  continuously for several minutes and does not track position.\n"
		"  Ctrl-C aborts and stops the motor.\n";
}

bool parse_args(int argc, char** argv, Args& args) {
	try {
		for (int i = 1; i < argc; ++i) {
			if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
				args.config_path = argv[++i];
			} else if (std::strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
				args.output_dir = argv[++i];
			} else if (std::strcmp(argv[i], "--rps-min") == 0 && i + 1 < argc) {
				args.rps_min = std::stof(argv[++i]);
			} else if (std::strcmp(argv[i], "--rps-max") == 0 && i + 1 < argc) {
				args.rps_max = std::stof(argv[++i]);
			} else if (std::strcmp(argv[i], "--rps-step") == 0 && i + 1 < argc) {
				args.rps_step = std::stof(argv[++i]);
			} else if (std::strcmp(argv[i], "--hold-s") == 0 && i + 1 < argc) {
				args.hold_s = std::stof(argv[++i]);
			} else if (std::strcmp(argv[i], "--settle-tol") == 0 && i + 1 < argc) {
				args.settle_tol = std::stof(argv[++i]);
			} else if (std::strcmp(argv[i], "--settle-hold-s") == 0 && i + 1 < argc) {
				args.settle_hold_s = std::stof(argv[++i]);
			} else if (std::strcmp(argv[i], "--sample-hz") == 0 && i + 1 < argc) {
				args.sample_hz = std::stof(argv[++i]);
			} else if (std::strcmp(argv[i], "--torque-constant") == 0 && i + 1 < argc) {
				args.torque_constant = std::stof(argv[++i]);
			} else if (std::strcmp(argv[i], "--max-step-wait-s") == 0 && i + 1 < argc) {
				args.max_step_wait_s = std::stof(argv[++i]);
			} else if (std::strcmp(argv[i], "--near-tol") == 0 && i + 1 < argc) {
				args.near_tol = std::stof(argv[++i]);
			} else if (std::strcmp(argv[i], "--grace-s") == 0 && i + 1 < argc) {
				args.grace_s = std::stof(argv[++i]);
			} else if (std::strcmp(argv[i], "--write-config") == 0) {
				args.write_config = true;
			} else if (std::strcmp(argv[i], "--verbose") == 0) {
				args.verbose = true;
			} else if (std::strcmp(argv[i], "--help") == 0 || std::strcmp(argv[i], "-h") == 0) {
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
	return true;
}

// One instantaneous sample logged during a ramp or post-settle dwell.
struct Sample {
	double t_s = 0.0; // seconds since sweep start
	int trial = 0;    // 1 = cumulative ramp, 2 = reset-to-zero each step
	int step_index = 0;
	std::string direction; // "up" or "down"
	float target_turns_s = 0.0f;
	float measured_turns_s = 0.0f;
	float iq_measured_a = 0.0f;
	bool iq_valid = false;
};

// One up/down transition summary (drives the timing cross-check regression).
struct StepResult {
	int trial = 0;
	int step_index = 0;
	std::string direction;
	float from_turns_s = 0.0f;
	float to_turns_s = 0.0f;
	float settle_time_s = 0.0f; // time to first enter and hold the tolerance band
	// Last velocity actually measured for this step — the real value even
	// when ok is false, so a failed step's "true" endpoint is known instead
	// of blindly assuming the nominal target was reached.
	float end_measured_turns_s = 0.0f;
	// True if this step needed the --grace-s extension to settle (or still
	// failed even with it) — settle_time_s already reflects the real total
	// time either way, this just flags that it wasn't a first-attempt settle.
	bool used_grace = false;
	bool ok = false;
};

double now_s(std::chrono::steady_clock::time_point t0) {
	return std::chrono::duration<double>(
		std::chrono::steady_clock::now() - t0).count();
}

std::string timestamp_label() {
	const auto now = std::chrono::system_clock::now();
	const std::time_t tt = std::chrono::system_clock::to_time_t(now);
	std::tm tm_buf{};
	localtime_r(&tt, &tm_buf);
	char buf[32];
	std::strftime(buf, sizeof(buf), "%Y-%m-%d_%H%M%S", &tm_buf);
	return buf;
}

// Command `target_turns_s` continuously (feeding the ODrive watchdog and
// sampling velocity/Iq) until velocity stays within settle_tol of target for
// settle_hold_s straight, then keeps commanding+logging for another hold_s.
// Returns false (step_out.ok stays false) if Ctrl-C fires or max_step_wait_s
// is exceeded before settling.
bool spin_to(PreyMotor& motor, const Args& args,
	std::chrono::steady_clock::time_point t0, int trial, int step_index,
	const std::string& direction, float from_turns_s, float target_turns_s,
	std::vector<Sample>& samples, StepResult& step_out,
	int& iq_success_count, int& iq_total_count)
{
	// Explicit field assignment (not positional aggregate-init) — safer
	// against StepResult growing new members out of declaration order.
	step_out = StepResult{};
	step_out.trial = trial;
	step_out.step_index = step_index;
	step_out.direction = direction;
	step_out.from_turns_s = from_turns_s;
	step_out.to_turns_s = target_turns_s;

	const auto step_start = std::chrono::steady_clock::now();
	const auto sample_period = std::chrono::duration<double>(1.0 / args.sample_hz);
	std::optional<double> in_band_since;
	std::optional<double> settled_at;
	float last_vel = from_turns_s;
	// Why: "timed out" alone can't tell oscillation/hunting (velocity swinging
	// through the band without holding) apart from a flat undershoot (stuck
	// short of target) or current saturation — track the attempt's range so
	// the diagnostic below can say which one actually happened.
	float min_vel = from_turns_s;
	float max_vel = from_turns_s;
	float max_iq_abs = 0.0f;
	float effective_wait_s = args.max_step_wait_s;
	bool grace_used = false;

	while (!g_stop.load()) {
		const auto iter_start = std::chrono::steady_clock::now();
		if (!motor.set_velocity_turns_s(target_turns_s)) {
			log_error("inertia_cal", "Set_Input_Vel failed");
			return false;
		}

		float vel_sample = 0.0f;
		if (motor.try_sample_velocity_turns_s(vel_sample, /*timeout_ms=*/0)) {
			last_vel = vel_sample;
			min_vel = std::min(min_vel, vel_sample);
			max_vel = std::max(max_vel, vel_sample);
		}
		float iq_sp = 0.0f;
		float iq_meas = 0.0f;
		++iq_total_count;
		const bool iq_ok = motor.try_get_iq(iq_sp, iq_meas);
		if (iq_ok) {
			++iq_success_count;
			max_iq_abs = std::max(max_iq_abs, std::fabs(iq_meas));
		}

		Sample s;
		s.t_s = now_s(t0);
		s.trial = trial;
		s.step_index = step_index;
		s.direction = direction;
		s.target_turns_s = target_turns_s;
		s.measured_turns_s = last_vel;
		s.iq_measured_a = iq_meas;
		s.iq_valid = iq_ok;
		samples.push_back(s);

		const double elapsed = std::chrono::duration<double>(
			std::chrono::steady_clock::now() - step_start).count();

		if (!settled_at) {
			if (std::fabs(last_vel - target_turns_s) <= args.settle_tol) {
				if (!in_band_since) {
					in_band_since = elapsed;
				} else if (elapsed - *in_band_since >= args.settle_hold_s) {
					settled_at = *in_band_since;
				}
			} else {
				in_band_since.reset();
			}
			// Why: check !settled_at again here, not just at loop entry —
			// settling can complete in *this same* iteration (the branch
			// above), and a stale wait-limit check must not override a
			// just-confirmed settle.
			if (!settled_at && elapsed > effective_wait_s) {
				// One grace_s extension if it's already close (--near-tol) —
				// a slow final approach is still real dynamics worth letting
				// finish, and whether it needed the extra time (and how much)
				// is itself useful signal for the regression, not just a
				// pass/fail bit.
				if (!grace_used
						&& std::fabs(last_vel - target_turns_s) <= args.near_tol) {
					grace_used = true;
					effective_wait_s += args.grace_s;
				} else {
					break;
				}
			}
		} else if (elapsed - *settled_at - args.settle_hold_s >= args.hold_s) {
			break; // finished the post-settle dwell
		}

		std::this_thread::sleep_until(iter_start
			+ std::chrono::duration_cast<std::chrono::steady_clock::duration>(
				sample_period));
	}

	step_out.end_measured_turns_s = last_vel;
	step_out.used_grace = grace_used;

	if (g_stop.load()) {
		return false;
	}
	if (!settled_at) {
		// Why: a bare "timed out" tells you nothing about whether this is a
		// resonance/back-EMF limit at this speed vs. an ODrive fault — dump
		// what we can so a stuck band doesn't require re-running with a
		// debugger attached.
		const float swing = max_vel - min_vel;
		const float end_offset = last_vel - target_turns_s;
		std::string pattern;
		if (swing > 2.0f * args.settle_tol) {
			// Why: crossing to both sides of the target is a genuinely
			// different failure mode (loop hunting) from moving a lot but
			// staying on one side (still ramping, or overshot and holding).
			pattern = (min_vel < target_turns_s && max_vel > target_turns_s)
				? "oscillating through the target (hunting)"
				: "swinging without settling";
		} else {
			// Not bouncing — say which side of the target it's actually
			// parked on and by how much, instead of assuming "short".
			char buf[64];
			std::snprintf(buf, sizeof(buf), "steady %s target by %.3f turns/s",
				(end_offset >= 0.0f ? "above" : "below"), std::fabs(end_offset));
			pattern = buf;
		}
		std::string diag = "Timed out reaching " + std::to_string(target_turns_s)
			+ " turns/s from " + std::to_string(from_turns_s) + " — " + pattern
			+ " (measured range " + std::to_string(min_vel) + " to "
			+ std::to_string(max_vel) + " turns/s, last " + std::to_string(last_vel)
			+ ", max |Iq| " + std::to_string(max_iq_abs) + " A)"
			+ (grace_used
				? " [still failed after +" + std::to_string(args.grace_s)
					+ "s grace extension]"
				: "");
		uint32_t active_errors = 0;
		uint32_t disarm_reason = 0;
		if (motor.try_get_active_errors(active_errors, disarm_reason)) {
			char buf[80];
			std::snprintf(buf, sizeof(buf),
				" | active_errors=0x%08X disarm_reason=0x%08X",
				active_errors, disarm_reason);
			diag += buf;
		}
		uint32_t axis_error = 0;
		uint32_t axis_state = 0;
		if (motor.read_axis_state(axis_error, axis_state)) {
			char buf[64];
			std::snprintf(buf, sizeof(buf), " | axis_error=0x%08X axis_state=%u",
				axis_error, axis_state);
			diag += buf;
		}
		log_error("inertia_cal", diag);
		return false;
	}
	step_out.settle_time_s = static_cast<float>(*settled_at);
	step_out.ok = true;
	return true;
}

// A step timing out at one speed (e.g. a mechanical resonance or back-EMF
// current-headroom limit at that specific RPM) shouldn't throw away every
// other data point in the sweep. Records the step either way and returns
// true only when the caller should stop the whole run: Ctrl-C, or enough
// consecutive failures in a row to look systemic (CAN drop, bad config)
// rather than a single bad speed.
constexpr int kMaxConsecutiveFailures = 5;

bool handle_step_result(const StepResult& step, std::vector<StepResult>& steps,
	int& consecutive_failures)
{
	steps.push_back(step);
	if (g_stop.load()) {
		return true;
	}
	if (step.ok) {
		consecutive_failures = 0;
		return false;
	}
	++consecutive_failures;
	if (consecutive_failures >= kMaxConsecutiveFailures) {
		log_error("inertia_cal", std::to_string(consecutive_failures)
			+ " consecutive step failures — aborting (this looks systemic, not "
			"a single bad speed; check CAN connection/heartbeat)");
		return true;
	}
	log_error("inertia_cal", "Skipping this target and continuing the sweep ("
		+ std::to_string(consecutive_failures) + " consecutive failure(s) so far)");
	return false;
}

std::vector<float> build_targets(float rps_min, float rps_max, float rps_step) {
	std::vector<float> out;
	if (rps_step <= 0.0f || rps_max < rps_min) {
		return out;
	}
	const int n = static_cast<int>(std::lround((rps_max - rps_min) / rps_step));
	for (int i = 0; i <= n; ++i) {
		out.push_back(rps_min + static_cast<float>(i) * rps_step);
	}
	return out;
}

double det3(const double m[3][3]) {
	return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
		- m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
		+ m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

// Closed-form least squares (3x3 normal equations via Cramer's rule).
bool solve3x3(const double A[3][3], const double b[3], double x[3]) {
	const double d = det3(A);
	if (std::fabs(d) < 1e-15) {
		return false;
	}
	for (int col = 0; col < 3; ++col) {
		double m[3][3];
		for (int r = 0; r < 3; ++r) {
			for (int c = 0; c < 3; ++c) {
				m[r][c] = (c == col) ? b[r] : A[r][c];
			}
		}
		x[col] = det3(m) / d;
	}
	return true;
}

// Fit torque_nm = a0*alpha_rad + a1*omega_rad + a2*sign(omega_rad).
struct DynamicFit {
	double j_kg_m2 = 0.0;
	double b_nm_s_per_rad = 0.0;
	double tau_c_nm = 0.0;
	double r_squared = 0.0;
	int n = 0;
};

DynamicFit fit_dynamic_model(const std::vector<Sample>& samples, float torque_constant,
	float sample_hz)
{
	DynamicFit result;
	// Why: consecutive samples should be ~1/sample_hz apart; a gap several
	// times that (scheduler hiccup, or a slow --sample-hz) means this pair
	// isn't a clean instantaneous derivative — exclude it rather than using a
	// fixed 0.1s cutoff that silently drops every row when --sample-hz < ~20.
	const double max_dt = std::max(0.05, 5.0 / std::max(1.0f, sample_hz));
	struct Row { double alpha_rad, omega_rad, sign_omega, torque_nm; };
	std::vector<Row> rows;
	for (size_t i = 0; i + 1 < samples.size(); ++i) {
		const Sample& a = samples[i];
		const Sample& b = samples[i + 1];
		if (a.trial != b.trial || a.step_index != b.step_index
				|| a.direction != b.direction || !a.iq_valid || !b.iq_valid) {
			continue; // don't differentiate across step boundaries or bad Iq reads
		}
		const double dt = b.t_s - a.t_s;
		if (dt <= 1e-4 || dt > max_dt) {
			continue;
		}
		const double accel_turns_s2 = (b.measured_turns_s - a.measured_turns_s) / dt;
		const double omega_turns_s = 0.5 * (a.measured_turns_s + b.measured_turns_s);
		const double iq = 0.5 * (a.iq_measured_a + b.iq_measured_a);
		Row row;
		row.alpha_rad = accel_turns_s2 * kTwoPi;
		row.omega_rad = omega_turns_s * kTwoPi;
		row.sign_omega = (row.omega_rad > 1e-3) ? 1.0
			: (row.omega_rad < -1e-3 ? -1.0 : 0.0);
		row.torque_nm = iq * torque_constant;
		rows.push_back(row);
	}
	result.n = static_cast<int>(rows.size());
	if (result.n < 10) {
		return result;
	}

	double A[3][3] = {};
	double b[3] = {};
	for (const auto& r : rows) {
		const double x[3] = {r.alpha_rad, r.omega_rad, r.sign_omega};
		for (int i = 0; i < 3; ++i) {
			for (int j = 0; j < 3; ++j) {
				A[i][j] += x[i] * x[j];
			}
			b[i] += x[i] * r.torque_nm;
		}
	}
	double x[3] = {};
	if (!solve3x3(A, b, x)) {
		result.n = 0;
		return result;
	}
	result.j_kg_m2 = x[0];
	result.b_nm_s_per_rad = x[1];
	result.tau_c_nm = x[2];

	double mean_y = 0.0;
	for (const auto& r : rows) {
		mean_y += r.torque_nm;
	}
	mean_y /= static_cast<double>(rows.size());
	double ss_tot = 0.0;
	double ss_res = 0.0;
	for (const auto& r : rows) {
		const double pred = result.j_kg_m2 * r.alpha_rad
			+ result.b_nm_s_per_rad * r.omega_rad + result.tau_c_nm * r.sign_omega;
		ss_res += (r.torque_nm - pred) * (r.torque_nm - pred);
		ss_tot += (r.torque_nm - mean_y) * (r.torque_nm - mean_y);
	}
	result.r_squared = (ss_tot > 1e-12) ? (1.0 - ss_res / ss_tot) : 0.0;
	return result;
}

// Cross-check only: settle_time_s ~= a + b*|delta_turns_s| (least squares).
struct TimingFit {
	double a_s = 0.0;
	double b_s_per_turn = 0.0;
	int n = 0;
};

TimingFit fit_timing_model(const std::vector<StepResult>& steps) {
	TimingFit fit;
	double sum_x = 0.0;
	double sum_y = 0.0;
	double sum_xx = 0.0;
	double sum_xy = 0.0;
	int n = 0;
	for (const auto& s : steps) {
		if (!s.ok) {
			continue;
		}
		const double x = std::fabs(s.to_turns_s - s.from_turns_s);
		const double y = s.settle_time_s;
		sum_x += x;
		sum_y += y;
		sum_xx += x * x;
		sum_xy += x * y;
		++n;
	}
	fit.n = n;
	if (n < 2) {
		return fit;
	}
	const double denom = n * sum_xx - sum_x * sum_x;
	if (std::fabs(denom) < 1e-9) {
		return fit;
	}
	fit.b_s_per_turn = (n * sum_xy - sum_x * sum_y) / denom;
	fit.a_s = (sum_y - fit.b_s_per_turn * sum_x) / n;
	return fit;
}

bool write_samples_csv(const std::string& path, const std::vector<Sample>& samples) {
	std::ofstream f(path);
	if (!f) {
		return false;
	}
	f << "t_s,trial,step_index,direction,target_turns_s,measured_turns_s,"
		"iq_measured_a,iq_valid\n";
	f << std::fixed << std::setprecision(5);
	for (const auto& s : samples) {
		f << s.t_s << ',' << s.trial << ',' << s.step_index << ',' << s.direction
			<< ',' << s.target_turns_s << ',' << s.measured_turns_s << ','
			<< s.iq_measured_a << ',' << (s.iq_valid ? 1 : 0) << '\n';
	}
	return true;
}

bool write_steps_csv(const std::string& path, const std::vector<StepResult>& steps) {
	std::ofstream f(path);
	if (!f) {
		return false;
	}
	f << "trial,step_index,direction,from_turns_s,to_turns_s,delta_turns_s,"
		"settle_time_s,end_measured_turns_s,used_grace,ok\n";
	f << std::fixed << std::setprecision(5);
	for (const auto& s : steps) {
		f << s.trial << ',' << s.step_index << ',' << s.direction << ','
			<< s.from_turns_s << ',' << s.to_turns_s << ','
			<< (s.to_turns_s - s.from_turns_s) << ',' << s.settle_time_s << ','
			<< s.end_measured_turns_s << ',' << (s.used_grace ? 1 : 0) << ','
			<< (s.ok ? 1 : 0) << '\n';
	}
	return true;
}

} // namespace

int main(int argc, char** argv) {
	Args args;
	if (!parse_args(argc, argv, args)) {
		print_usage();
		return 1;
	}
	if (args.rps_min <= 0.0f || args.rps_max <= args.rps_min || args.rps_step <= 0.0f) {
		std::cerr << "Invalid --rps-min/--rps-max/--rps-step\n";
		return 1;
	}
	// Why: sample_hz feeds a 1.0/sample_hz sleep period below — <= 0 would
	// divide by zero and hang the sweep with the motor still spinning.
	if (args.sample_hz <= 0.0f) {
		std::cerr << "--sample-hz must be > 0\n";
		return 1;
	}
	if (args.settle_tol <= 0.0f) {
		std::cerr << "--settle-tol must be > 0\n";
		return 1;
	}
	if (args.settle_hold_s < 0.0f || args.hold_s < 0.0f) {
		std::cerr << "--settle-hold-s/--hold-s must be >= 0\n";
		return 1;
	}
	if (args.max_step_wait_s <= 0.0f) {
		std::cerr << "--max-step-wait-s must be > 0\n";
		return 1;
	}
	if (args.torque_constant <= 0.0f) {
		std::cerr << "--torque-constant must be > 0\n";
		return 1;
	}
	if (args.near_tol < 0.0f || args.grace_s < 0.0f) {
		std::cerr << "--near-tol/--grace-s must be >= 0\n";
		return 1;
	}

	Logger::instance().set_level(args.verbose ? LogLevel::Debug : LogLevel::Info);

	const std::string config_path = resolve_arena_config_path(argv[0], args.config_path);
	if (config_path.empty()) {
		log_error("inertia_cal", "No arena config found — pass --config <path>");
		return 1;
	}
	ArenaExperimentConfig cfg;
	if (!load_arena_experiment_config(config_path, cfg)) {
		return 1;
	}
	if (cfg.motor.chain_mm_per_motor_turn <= 0.0f && cfg.motor.pulley_radius_m <= 0.0f) {
		log_error("inertia_cal", "motor chain scale is 0 in " + config_path);
		return 1;
	}
	if (!can_interface_up(cfg.motor.can_interface)) {
		log_error("inertia_cal", can_interface_down_hint(cfg.motor.can_interface));
		return 1;
	}

	// Why: measure the true current-limited step response, not one partially
	// closed by a previous calibration's torque_ff — re-running this test on
	// an already-calibrated rig would otherwise underestimate J/B/tau_c,
	// since feed-forward would already be doing some of the work.
	PreyMotorConfig motor_cfg = prey_motor_from_config(cfg.motor);
	motor_cfg.chain_inertia_kg_m2 = 0.0f;
	motor_cfg.chain_viscous_friction_nm_s_per_rad = 0.0f;
	motor_cfg.chain_static_friction_nm = 0.0f;
	PreyMotor motor(motor_cfg);
	g_motor = &motor;
	std::signal(SIGINT, signal_handler);
	std::signal(SIGTERM, signal_handler);

	if (!motor.connect() || !motor.status().heartbeat_ok) {
		log_error("inertia_cal", "Prey motor connect/heartbeat failed");
		return 1;
	}
	if (!motor.enter_velocity_mode()) {
		log_error("inertia_cal", "Failed to enter closed-loop velocity mode");
		return 1;
	}

	const std::vector<float> targets = build_targets(args.rps_min, args.rps_max,
		args.rps_step);
	std::cout << "Sweeping " << targets.size() << " targets from " << args.rps_min
		<< " to " << args.rps_max << " turns/s (step " << args.rps_step << ").\n"
		<< "Settle: within +/-" << args.settle_tol << " turns/s held "
		<< args.settle_hold_s << "s; dwell " << args.hold_s << "s after settling.\n"
		<< "The chain must be a closed loop — this does not track position.\n"
		<< "Ctrl-C to abort.\n";

	std::vector<Sample> samples;
	std::vector<StepResult> steps;
	int iq_success_count = 0;
	int iq_total_count = 0;
	const auto t0 = std::chrono::steady_clock::now();
	bool aborted = false;

	// A step that fails to settle (resonance/current-limit pocket at that
	// speed, etc.) is recorded and skipped rather than aborting the whole
	// sweep — see handle_step_result. consecutive_failures is shared across
	// both trials so a systemic problem (not just one bad speed) still stops
	// the run instead of burning through the rest at --max-step-wait-s each.
	int consecutive_failures = 0;

	// --- Trial A: cumulative ramp (spin from wherever it already is) ---
	float cur = 0.0f;
	for (size_t i = 0; i < targets.size() && !aborted; ++i) {
		StepResult step;
		spin_to(motor, args, t0, /*trial=*/1, static_cast<int>(i), "up",
			cur, targets[i], samples, step, iq_success_count, iq_total_count);
		cur = step.ok ? targets[i] : step.end_measured_turns_s;
		if (handle_step_result(step, steps, consecutive_failures)) {
			aborted = true;
			break;
		}
	}
	if (!aborted) {
		StepResult step;
		spin_to(motor, args, t0, /*trial=*/1, static_cast<int>(targets.size()),
			"down", cur, 0.0f, samples, step, iq_success_count, iq_total_count);
		if (handle_step_result(step, steps, consecutive_failures)) {
			aborted = true;
		}
	}

	// --- Trial B: reset to zero before every step ---
	for (size_t i = 0; i < targets.size() && !aborted; ++i) {
		StepResult up;
		spin_to(motor, args, t0, /*trial=*/2, static_cast<int>(i), "up",
			0.0f, targets[i], samples, up, iq_success_count, iq_total_count);
		if (handle_step_result(up, steps, consecutive_failures)) {
			aborted = true;
			break;
		}

		// Why: spin_to always commands the motor back toward 0 next, so
		// attempt the descent regardless of whether "up" actually settled —
		// leaving the motor at speed because one step failed is worse than
		// one extra (likely also failing) data point.
		StepResult down;
		spin_to(motor, args, t0, /*trial=*/2, static_cast<int>(i), "down",
			up.ok ? targets[i] : up.end_measured_turns_s, 0.0f, samples, down,
			iq_success_count, iq_total_count);
		if (handle_step_result(down, steps, consecutive_failures)) {
			aborted = true;
			break;
		}
	}

	motor.stop();

	const std::string out_dir = args.output_dir + "/motor_inertia_calibration/"
		+ timestamp_label();
	std::filesystem::create_directories(out_dir);
	const bool samples_written = write_samples_csv(out_dir + "/samples.csv", samples);
	const bool steps_written = write_steps_csv(out_dir + "/steps.csv", steps);
	if (!samples_written) {
		log_error("inertia_cal", "Failed to write " + out_dir + "/samples.csv");
	}
	if (!steps_written) {
		log_error("inertia_cal", "Failed to write " + out_dir + "/steps.csv");
	}
	const int failed_steps = static_cast<int>(std::count_if(steps.begin(), steps.end(),
		[](const StepResult& s) { return !s.ok; }));
	std::cout << "Wrote " << samples.size() << " samples, " << steps.size()
		<< " step summaries (" << (steps.size() - failed_steps) << " ok, "
		<< failed_steps << " failed/timed out) to " << out_dir << "\n";
	if (failed_steps > 0) {
		std::cout << failed_steps << " step(s) never settled — see the "
			"log lines above and steps.csv (ok=0 rows) for which speeds. A "
			"narrow band that consistently fails while its neighbors settle "
			"fine usually means a real mechanical/electrical limit at that "
			"RPM (resonance, or current saturating against back-EMF), not a "
			"test bug — check iq_measured_a in samples.csv near the failed "
			"target for saturation against the current limit.\n";
	}

	if (aborted) {
		log_error("inertia_cal",
			"Sweep aborted — partial data written, no regression run");
		return 1;
	}

	const double iq_rate = iq_total_count > 0
		? static_cast<double>(iq_success_count) / iq_total_count : 0.0;
	if (iq_rate < 0.5) {
		log_error("inertia_cal", "Only " + std::to_string(iq_rate * 100.0)
			+ "% of Get_Iq reads succeeded — enable cyclic Iq broadcast in the "
			"ODrive's CAN config (odrivetool) before trusting the dynamic fit "
			"below; the timing cross-check is unaffected");
	}

	const TimingFit timing = fit_timing_model(steps);
	std::cout << std::fixed << std::setprecision(6)
		<< "\n=== Timing cross-check (settle_time_s ~= a + b*|delta_turns_s|) ===\n"
		<< "n=" << timing.n << "  a=" << timing.a_s << " s  b=" << timing.b_s_per_turn
		<< " s per turn/s\n";

	const DynamicFit dyn = fit_dynamic_model(samples, args.torque_constant, args.sample_hz);
	if (dyn.n < 10) {
		log_error("inertia_cal",
			"Not enough valid sample pairs (Iq + consecutive velocity) for the "
			"dynamic fit — check Iq telemetry above");
		return 1;
	}
	if (dyn.j_kg_m2 <= 0.0) {
		log_error("inertia_cal", "Fitted chain_inertia_kg_m2 is <= 0 — physically "
			"implausible, treat this run's numbers as unreliable (check Iq "
			"telemetry, torque constant, and settle tolerance)");
	}

	std::cout << "\n=== Motor inertia calibration result ===\n"
		<< "Torque constant used: " << args.torque_constant << " N*m/A\n"
		<< "Samples used in dynamic fit: " << dyn.n << "   R^2: " << dyn.r_squared << "\n"
		<< "chain_inertia_kg_m2 = " << dyn.j_kg_m2 << "\n"
		<< "chain_viscous_friction_nm_s_per_rad = " << dyn.b_nm_s_per_rad << "\n"
		<< "chain_static_friction_nm = " << dyn.tau_c_nm << "\n"
		<< "\nPaste into config/arena_experiment.json \"motor\" section"
		<< (args.write_config ? " (also writing automatically below):\n" : ":\n")
		<< "  \"chain_inertia_kg_m2\": " << dyn.j_kg_m2 << ",\n"
		<< "  \"chain_viscous_friction_nm_s_per_rad\": " << dyn.b_nm_s_per_rad << ",\n"
		<< "  \"chain_static_friction_nm\": " << dyn.tau_c_nm << ",\n"
		<< "  \"torque_constant_nm_per_a\": " << args.torque_constant << "\n";

	if (args.write_config) {
		MotorConfig updated = cfg.motor;
		updated.chain_inertia_kg_m2 = static_cast<float>(dyn.j_kg_m2);
		updated.chain_viscous_friction_nm_s_per_rad =
			static_cast<float>(dyn.b_nm_s_per_rad);
		updated.chain_static_friction_nm = static_cast<float>(dyn.tau_c_nm);
		updated.torque_constant_nm_per_a = args.torque_constant;
		if (!save_motor_calibration(config_path, updated)) {
			log_error("inertia_cal", "Failed to write calibration into " + config_path);
			return 1;
		}
		std::cout << "Wrote calibration into " << config_path << "\n";
	}

	return 0;
}
