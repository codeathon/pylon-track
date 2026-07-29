#pragma once

#include <string>

// Resolve arena_experiment.json beside executable or under config/.
std::string resolve_arena_config_path(const char* argv0, const std::string& user_path);

// Default calib.npz beside executable or repo root.
std::string resolve_calib_output_path(const char* argv0, const std::string& user_path);

bool can_interface_up(const std::string& iface);

void print_setup_banner(const char* step_name, int step_num, int step_total);

bool prompt_enter(const char* message);
bool prompt_yes_no(const char* message);
float prompt_float(const char* message);
