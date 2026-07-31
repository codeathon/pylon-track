#pragma once

#include <optional>
#include <string>

#include "calibrate/setup_options.h"

// Resolve arena_experiment.json beside executable or under config/.
std::string resolve_arena_config_path(const char* argv0, const std::string& user_path);

// Default calib.npz beside executable or repo root.
std::string resolve_calib_output_path(const char* argv0, const std::string& user_path);

bool can_interface_up(const std::string& iface);

void print_setup_banner(const char* step_name, int step_num, int step_total);

bool prompt_enter(const char* message);

// nullopt means stdin closed/failed (EOF) — distinct from a real "n" or "0"
// answer, so callers can abort instead of silently treating it as valid input.
std::optional<bool> prompt_yes_no(const char* message);
std::optional<float> prompt_float(const char* message);

// Sleeps in small increments up to total_ms, checking opts.running each
// increment. Returns false (and stops early) if opts.running is set and
// becomes false — callers driving live hardware must stop the motor
// immediately when this returns false rather than finishing the sleep.
bool interruptible_sleep_ms(const SetupOptions& opts, int total_ms);
