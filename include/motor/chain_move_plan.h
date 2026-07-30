#pragma once

#include <algorithm>
#include <cmath>
#include <string>

// Planning inputs/outputs for a chain distance-in-time move.
// Why: one checklist for which parameters make a move feasible and accurate.
struct ChainMovePlan {
	// Inputs
	float distance_mm = 0.0f;
	int duration_ms = 0;
	float accel_mps2 = 5.0f;
	float mm_per_turn = 0.0f; // chain_mm_per_motor_turn (or pulley-derived)
	int chain_direction_sign = 1;
	float odrive_vel_limit_turns_s = 10.0f; // matches Set_Limits default in ODriveCan

	// Derived profile
	float duration_s = 0.0f;
	float avg_speed_mps = 0.0f;
	float peak_speed_mps = 0.0f;
	float peak_turns_s = 0.0f; // Set_Input_Vel PreyMotor will send at peak
	float accel_time_s = 0.0f;
	float cruise_time_s = 0.0f;
	float expected_distance_mm = 0.0f;
	float min_accel_mps2 = 0.0f; // accel needed to cover full distance in time

	bool scale_ok = false;
	bool timing_ok = false;
	bool accel_ok = false;
	bool vel_limit_ok = false;
	bool feasible = false;

	std::string summary;
};

// Pure planning — no motor I/O. Same peak/accel math as MotionPlanner.
inline ChainMovePlan plan_chain_move(float distance_mm, int duration_ms,
	float accel_mps2, float mm_per_turn, int chain_direction_sign,
	float odrive_vel_limit_turns_s = 10.0f)
{
	ChainMovePlan p;
	p.distance_mm = distance_mm;
	p.duration_ms = duration_ms;
	p.accel_mps2 = (accel_mps2 > 0.0f) ? accel_mps2 : 0.5f;
	p.mm_per_turn = mm_per_turn;
	p.chain_direction_sign = (chain_direction_sign < 0) ? -1 : 1;
	p.odrive_vel_limit_turns_s = odrive_vel_limit_turns_s;

	p.scale_ok = mm_per_turn > 1e-3f;
	p.timing_ok = duration_ms > 0 && std::fabs(distance_mm) >= 1e-3f;
	if (!p.timing_ok) {
		p.summary = "[!!] Need non-zero --distance-mm and --duration-ms > 0\n";
		return p;
	}

	p.duration_s = static_cast<float>(duration_ms) / 1000.0f;
	const float distance_m = distance_mm / 1000.0f;
	const float abs_x = std::fabs(distance_m);
	const float T = p.duration_s;
	const float a = p.accel_mps2;
	p.avg_speed_mps = distance_m / T;

	// Triangle covering |x| in T needs a_min = 4|x|/T^2 (peak = 2|x|/T = a_min*T/2).
	p.min_accel_mps2 = 4.0f * abs_x / (T * T);
	p.accel_ok = a + 1e-6f >= p.min_accel_mps2;

	float peak_abs = 0.0f;
	if (p.accel_ok) {
		// Why: triangle peak 2|x|/T with cruise overshoots. Solve trapezoid
		// |x| = v*T - v^2/a  →  v = (aT - sqrt((aT)^2 - 4a|x|)) / 2.
		const float disc = std::max(0.0f, (a * T) * (a * T) - 4.0f * a * abs_x);
		peak_abs = 0.5f * (a * T - std::sqrt(disc));
	} else {
		// Best effort: bang-bang triangle at accel limit (short of |x|).
		peak_abs = a * T * 0.5f;
	}

	p.peak_speed_mps = std::copysign(peak_abs, distance_m);
	p.accel_time_s = (peak_abs > 1e-6f && a > 1e-6f) ? (peak_abs / a) : 0.0f;
	p.cruise_time_s = std::max(0.0f, T - 2.0f * p.accel_time_s);

	// Triangle: x = v*T/2. Trapezoid: x = v*(T - t_accel) = v*T - v^2/a.
	if (p.cruise_time_s <= 1e-6f) {
		p.expected_distance_mm = std::copysign(
			1000.0f * peak_abs * T * 0.5f, distance_mm);
	} else {
		p.expected_distance_mm = std::copysign(
			1000.0f * peak_abs * (T - p.accel_time_s), distance_mm);
	}

	if (p.scale_ok) {
		// Matches PreyMotor::chain_mps_to_turns_s(peak_speed_mps).
		p.peak_turns_s = (p.peak_speed_mps * 1000.0f / mm_per_turn)
			* static_cast<float>(p.chain_direction_sign);
	}
	p.vel_limit_ok = p.scale_ok
		&& std::fabs(p.peak_turns_s) <= odrive_vel_limit_turns_s + 1e-3f;

	const float dist_err = std::fabs(
		std::fabs(p.expected_distance_mm) - std::fabs(distance_mm));
	p.feasible = p.scale_ok && p.timing_ok && p.accel_ok && p.vel_limit_ok
		&& dist_err < 0.5f;

	p.summary.clear();
	p.summary += "=== Chain move plan ===\n";
	p.summary += "Request: " + std::to_string(distance_mm) + " mm in "
		+ std::to_string(duration_ms) + " ms\n";
	p.summary += p.scale_ok ? "[ok] " : "[!!] ";
	p.summary += "motor.chain_mm_per_motor_turn (or pulley) = "
		+ std::to_string(mm_per_turn) + " mm/turn\n";
	p.summary += "     motor.chain_direction_sign = "
		+ std::to_string(p.chain_direction_sign) + "\n";
	p.summary += p.accel_ok ? "[ok] " : "[!!] ";
	p.summary += "accel = " + std::to_string(p.accel_mps2)
		+ " m/s^2 (need >= " + std::to_string(p.min_accel_mps2)
		+ " to hit full distance in time)\n";
	p.summary += p.vel_limit_ok ? "[ok] " : "[!!] ";
	p.summary += "peak Set_Input_Vel = " + std::to_string(p.peak_turns_s)
		+ " turns/s (ODrive vel_limit " + std::to_string(odrive_vel_limit_turns_s)
		+ ")\n";
	p.summary += "Profile: t_accel=" + std::to_string(p.accel_time_s)
		+ "s  t_cruise=" + std::to_string(p.cruise_time_s)
		+ "s  peak_chain=" + std::to_string(p.peak_speed_mps * 1000.0f)
		+ " mm/s\n";
	p.summary += "Expected travel from profile: "
		+ std::to_string(p.expected_distance_mm) + " mm\n";
	p.summary += "\nODrive must also have (not in JSON):\n";
	p.summary += "  - motor+encoder calibration saved (pre_calibrated)\n";
	p.summary += "  - closed-loop + VELOCITY control (app sets over CAN)\n";
	p.summary += "  - vel_limit / current_limit high enough (app Set_Limits 10 / 40)\n";
	p.summary += "  - watchdog fed (app re-sends Set_Input_Vel ~50 Hz)\n";
	p.summary += p.feasible ? "\n[ok] PLAN FEASIBLE\n" : "\n[!!] PLAN NOT FEASIBLE\n";
	return p;
}
